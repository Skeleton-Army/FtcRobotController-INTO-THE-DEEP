package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.skeletonarmy.marrow.autonomous.AutoOpMode;
import com.skeletonarmy.marrow.actions.SleepUntilAction;
import com.skeletonarmy.marrow.autonomous.State;
import com.skeletonarmy.marrow.prompts.OptionPrompt;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;

enum Alliance {
    RED,
    BLUE
}

enum Strategy {
    SPECIMENS,
    BASKET
}

@Autonomous(name = "Autonomous App", group = "SA_FTC", preselectTeleOp = "Teleop App")
public class AutoApplication extends AutoOpMode {
    MecanumDrive drive;

    Intake intake;
    Outtake outtake;
    SpecimenArm specimenArm;

    Drive driveActions;

    Alliance alliance;
    Strategy strategy;
    int extraSpecimens;

    Pose2d startPose;
    WebcamCV camCV;

    DigitalChannel outtakeSwitch;

    int collectedSamples = 0;
    int hangedSpecimens = 0;

    boolean didCollectSamples = false;

    @Override
    public void preAutonomousSetup() {
        alliance = prompt(new OptionPrompt<>("SELECT AN ALLIANCE:", Alliance.RED, Alliance.BLUE));
        strategy = prompt(new OptionPrompt<>("SELECT A STRATEGY:", Strategy.SPECIMENS, Strategy.BASKET));
        extraSpecimens = 0;

        if (strategy == Strategy.SPECIMENS) {
            extraSpecimens = prompt(new OptionPrompt<>("SELECT HUMAN PLAYER SPECIMENS:", 1, 0));
        }

        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Strategy", strategy);
        telemetry.addData("Selected Specimen", extraSpecimens);

        startPose =
                strategy == Strategy.SPECIMENS
                ? new Pose2d(0, -62.5, Math.toRadians(90.00)) // Specimens
                : new Pose2d(-39, -62.5, Math.toRadians(0)); // Basket
    }

    @Override
    public void onStateMachineStart() {
        setFallbackState(() -> gamepad1.guide.isDown() || gamepad2.guide.isDown(), this::resetRobot);

        switch (strategy) {
            case SPECIMENS:
                transition("hangSpecimen");
            case BASKET:
                transition("putInBasket");
        }
    }

    @Override
    public void onInit() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);

        outtakeSwitch = hardwareMap.get(DigitalChannel.class, OuttakeConfig.limitSwitchName);

        runBlocking(specimenArm.grabClose());
        runBlocking(intake.wristMiddle());
        runBlocking(intake.rotate(0));
    }

    @Override
    public void onStart() {
        // Set starting position
        drive.pose = startPose;

        // Configure webcam CV
        camCV = new WebcamCV(hardwareMap, telemetry, drive);
        camCV.configureWebcam(new SampleColor[] { SampleColor.YELLOW, alliance == Alliance.RED ? SampleColor.RED : SampleColor.BLUE });

        driveActions = new Drive(drive, camCV, telemetry);
    }

    // -------------- States --------------

    @State(requiredTime = 2, timeoutState = "park")
    public void hangSpecimen() {
        hangedSpecimens++;

        int angleCompensation = (hangedSpecimens - 1) * -4;

        runBlocking(
                new ParallelAction(
                        specimenArm.grabClose(),
                        specimenArm.gripToOuttake(),
                        specimenArm.goToOuttake(),

                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(startPose.position.x, -37, Math.toRadians(95 + angleCompensation)), Math.PI / 2, null, new ProfileAccelConstraint(-1000000, hangedSpecimens == 1 ? 60 : 150))
                                .splineToLinearHeading(new Pose2d(startPose.position.x,  hangedSpecimens == 5 ? -30 : -32.5, Math.toRadians(95 + angleCompensation)), Math.PI / 2, null, new ProfileAccelConstraint(-60, hangedSpecimens == 1 ? 60 : 150))
                                .build()
                )
        );

        if (!didCollectSamples) {
            transition("collectColorSamples");
        } else if (hangedSpecimens < (4 + extraSpecimens)) {
            transition("collectSpecimen");
        } else {
            transition("park");
        }
    }

    @State(requiredTime = 2, timeoutState = "park")
    public void collectSpecimen() {
        int angleCompensation = (hangedSpecimens - 1) * -4;

        if (hangedSpecimens > 1) {
            runBlocking(
                    new ParallelAction(
                            specimenArm.goToHanged(),
                            specimenArm.grabOpen()
                    )
            );
        }

        runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(hangedSpecimens == 1 ? 180 : 270))
                                .splineToLinearHeading(new Pose2d(27, -63.2, Math.toRadians(95 + angleCompensation)), Math.toRadians(270), null, new ProfileAccelConstraint(-60, 150))
                                .build(),
                        new SequentialAction(
                                specimenArm.grabOpen(),
                                new SleepUntilAction(() -> drive.pose.position.y < -45),
                                specimenArm.gripToIntake(),
                                specimenArm.goToIntake()
                        )
                )
        );

        runBlocking(
                new SequentialAction(
                        specimenArm.grabClose(),
                        new SleepAction(0.25)
                )
        );

        transition("hangSpecimen");
    }

    @State
    public void collectColorSamples() {
        didCollectSamples = true;

        runAsync(
                new SequentialAction(
                        specimenArm.grabOpen(),
                        new SleepUntilAction(() -> drive.pose.position.y < -50),
                        specimenArm.gripToIntake(),
                        specimenArm.goToIntake()
                )
        );

        // Grab first sample
        runAsync(
                new SequentialAction(
                        new SleepUntilAction(() -> drive.pose.position.y < -42),
                        intake.wristReady(),
                        intake.extend(0.52)
                )
        );

        runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(48.5, -42, Math.toRadians(95)), 0, null, new ProfileAccelConstraint(-25, 100))
                                .build(),
                        new SequentialAction(
                                intake.openClaw(),
                                intake.extendWrist(),
                                new SleepAction(0.2),
                                intake.closeClaw(),
                                new SleepAction(0.1)
                        )
                )
        );

        // Grab second sample
        runBlocking(
                new ParallelAction(
                        // Put in basket
                        intake.retractWrist(),
                        outtake.hold(),
                        new SequentialAction(
                                new ParallelAction(
                                        intake.retract(),
                                        new SleepAction(0.6)
                                ),
                                intake.openClaw(),
                                intake.wristReady(),
                                new SleepAction(0.2)
                        ),

                        drive.actionBuilder(drive.pose)
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(58, -40.5), 0)
                                .build()
                )
        );

        runBlocking(
                new ParallelAction(
                        // Drop sample
                        new SequentialAction(
                                outtake.bucketMiddle(),
                                new ParallelAction(
                                        outtake.extend(0.3),
                                        new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -200),
                                        outtake.dunk()
                                ),
                                new SleepAction(0.3),
                                outtake.hold(),
                                outtake.retract()
                        ),

                        // Grab sample
                        new SequentialAction(
                                intake.wristReady(),
                                intake.openClaw(),
                                intake.extend(0.52),
                                new SleepAction(0.3), // Wait for sample to fall out
                                intake.extendWrist(),
                                new SleepAction(0.2),
                                intake.closeClaw(),
                                new SleepAction(0.1),
                                intake.wristReady()
                        )
                )
        );

        // Grab third sample
        runBlocking(
                new SequentialAction(
                        // Put in basket
                        intake.retractWrist(),
                        outtake.hold(),
                        new SequentialAction(
                                new ParallelAction(
                                        intake.retract(),
                                        new SleepAction(0.6)
                                ),
                                intake.openClaw(),
                                intake.wristReady(),
                                new SleepAction(0.2)
                        ),

                        // Drop sample
                        new SequentialAction(
                                outtake.bucketMiddle(),
                                new ParallelAction(
                                        outtake.extend(0.3),
                                        new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -200),
                                        outtake.dunk()
                                ),
                                new SleepAction(0.3)
                        ),

                        new ParallelAction(
                                outtake.hold(),
                                outtake.retract(),

                                drive.actionBuilder(drive.pose)
                                        .setTangent(0)
                                        .splineToLinearHeading(new Pose2d(56, -41.5, Math.toRadians(60)), 0)
                                        .build(),

                                // Grab sample
                                new SequentialAction(
                                        intake.wristReady(),
                                        intake.rotate(-0.2),
                                        intake.extraOpenClaw(),
                                        intake.extend(0.67),
                                        new SleepAction(0.2),
                                        intake.extendWrist(),
                                        new SleepAction(0.2),
                                        intake.closeClaw(),
                                        new SleepAction(0.1),
                                        intake.wristReady()
                                )
                        ),

                        new ParallelAction(
                                drive.actionBuilder(drive.pose)
                                        .splineToLinearHeading(new Pose2d(57.5, -39, Math.toRadians(95)), 0)
                                        .build(),

                                // Put in basket
                                intake.retractWrist(),
                                outtake.hold(),
                                new SequentialAction(
                                        new ParallelAction(
                                                intake.retract(),
                                                new SleepAction(0.6)
                                        ),
                                        intake.openClaw(),
                                        intake.wristMiddle(),
                                        new SleepAction(0.2)
                                )
                        ),

                        // Drop sample
                        new SequentialAction(
                                outtake.bucketMiddle(),
                                new ParallelAction(
                                        outtake.extend(0.3),
                                        new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -200),
                                        outtake.dunk()
                                )
                        )
                )
        );

        runAsync(
                new SequentialAction(
                        new SleepAction(0.3),
                        outtake.hold(),
                        outtake.retract()
                )
        );

        transition("collectSpecimen");
    }

    @State
    public void collectYellowSample() {
        collectedSamples++;

        Action wristSequence = new SequentialAction(
                intake.extendWrist()
//                new SleepAction(0.15)
        );

        runAsync(intake.openClaw());
        runAsync(intake.rotate(0));

        switch (collectedSamples) {
            case 1:
                // Collect first sample
                runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .afterDisp(4.5, wristSequence)
                                        .splineToLinearHeading(new Pose2d(-51, -50.5, Math.toRadians(80)), Math.PI)
                                        .build()
                        )
                );
                break;
            case 2:
                // Collect second sample
                runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .afterDisp(7, wristSequence)
                                        .splineToLinearHeading(new Pose2d(-56, -49.5, Math.toRadians(90)), Math.PI)
                                        .build()
                        )
                );
                break;
            case 3:
                // Collect third sample
                runAsync(
                        intake.rotate(0.2)
                );

                runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .afterDisp(9, wristSequence)
                                        .splineToLinearHeading(new Pose2d(-54.5, -46.5, Math.toRadians(120)), Math.PI)
                                        .build()
                        )
                );
                break;
        }

        transition("putInBasket");
    }

    @State(requiredTime = 2, timeoutState = "park")
    public void putInBasket() {
        Action grab = new SequentialAction(
                intake.closeClaw(),
                new SleepAction(0.1)
        );

        Action intakeRetract = new SequentialAction(
                outtake.hold(),
                intake.retractWrist(),
                intake.rotate(0),
                intake.retract(),
                intake.openClaw(),
                new SleepAction(0.05),
                intake.wristReady(),
                new SleepAction(0.2)
        );

        Action extendOuttake = new ParallelAction(
                outtake.bucketMiddle(),
                outtake.extend(),
                new SequentialAction(
                        new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -400),
                        outtake.bucketReady()
                )
        );

        Action dunk = new SequentialAction(
                new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -850),
                outtake.dunk(),
                new SleepAction(0.25)
        );

        Action dunkSequence = new ParallelAction(
                extendOuttake,
                dunk
        );

        if (collectedSamples == 0) {
            runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.PI / 2)
                                    .splineToLinearHeading(new Pose2d(-56, -54.5, Math.toRadians(45)), Math.toRadians(225), null, new ProfileAccelConstraint(-100, 200))
                                    .build(),
                            dunkSequence
                    )
            );
        }
        else if (collectedSamples >= 4) {
            double xCompensation = collectedSamples >= 6 ? 3 : 0;
            double yCompensation = collectedSamples >= 6 ? 3 : 0;
            double angleCompensation = 0;

            if (collectedSamples == 6) angleCompensation = 10;
            if (collectedSamples == 7) angleCompensation = 15;

            runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(200))
                                    .splineToLinearHeading(new Pose2d(-55.5 + xCompensation, -55 - yCompensation, Math.toRadians(45 + angleCompensation)), Math.toRadians(225), null, new ProfileAccelConstraint(-80, 200))
                                    .build(),
                            new SequentialAction(
                                    intakeRetract,
                                    dunkSequence
                            )
                    )
            );
        }
        else {
            // Grab sample
            runBlocking(grab);

            // Retract and go to basket
            runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose)
                                    .splineToLinearHeading(new Pose2d(-55.5, -56.5, Math.toRadians(45)), Math.toRadians(225))
                                    .build(),
                            new SequentialAction(
                                    intakeRetract,
                                    dunkSequence
                            )
                    )
            );
        }

        // Put the sample in the basket
        if (collectedSamples < 3) {
            runAsync(
                    new ParallelAction(
                            intake.openClaw(),
                            intake.extend(0.9),
                            intake.wristReady()
                    )
            );
        }

        runAsync(
                new SequentialAction(
                        new SleepAction(0.3),
                        outtake.hold(),
                        outtake.retract()
                )
        );

        if (collectedSamples < 6) {
            conditionalTransition(collectedSamples < 3, "collectYellowSample", "sampleFromSubmersible");
        } else {
            transition("park");
        }
    }

    @State(requiredTime = 4.5, timeoutState = "park", forceExitTime = 1)
    public void sampleFromSubmersible() {
        collectedSamples++;

        Action extendSequence = new SequentialAction(
                intake.wristReady(),
                intake.openClaw(),
                new ParallelAction(
                        intake.extend()
//                        new SequentialAction(
//                                new SleepUntilAction(() -> intake.motor.getCurrentPosition() <= -400),
//                                intake.extendWrist()
//                        )
                )
//                new SleepAction(0.1)
        );

        Action grabSequence = new SequentialAction(
                intake.extendWrist(),
                new SleepAction(0.3),
                intake.closeClaw(),
                new SleepAction(0.1)
//                intake.retractWrist()
//                new SleepAction(0.1)
        );

//        if (collectedSamples == 5) camCV.startStream();

        double xCompensation = collectedSamples >= 6 ? 2 : 0;

        runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(-32 - xCompensation, -5), Math.toRadians(0), null, new ProfileAccelConstraint(-100, 200))
                        .build()
        );

        runBlocking(
                new SleepAction(0.4)
        );

        Vector2d lower = new Vector2d(-8, -10);
        Vector2d upper = new Vector2d(5, 8);

        camCV.resetSampleList();

        // TODO: Add some sort of validation For example if (bad == yes): don't. This is here because funny. There will never be any validation, deal with it.

        // Grab sample
        runBlocking(
                new SleepUntilAction(() -> camCV.lookForSamples())
        );

        Vector2d bestSamplePos = new Vector2d(-3, drive.pose.position.y + CameraConfig.pickupSampleOffsetX);

        Sample targetSample = camCV.getBestSampleInRange(bestSamplePos, lower, upper);

        if (targetSample == null) targetSample = camCV.getBestSample(bestSamplePos);

        //        Sample targetSample = camCV.getBestSample(new Vector2d(-3, drive.pose.position.y + CameraConfig.pickupSampleOffsetX));

//        telemetry.addData("Target Sample", targetSample.getSamplePosition().position);

        double orientation = -targetSample.orientation;
        double normalizedOrientation = (90 - Math.abs(orientation)) * Math.signum(orientation);
        double rotationTarget = normalizedOrientation / 90;

//        double wiggleX = Math.sin(Math.toRadians(normalizedOrientation)) * wiggleDistance;
//        double wiggleY = Math.cos(Math.toRadians(normalizedOrientation)) * wiggleDistance;
//        double wiggleBackX = Math.sin(Math.toRadians(normalizedOrientation)) * wiggleBackDistance;
//        double wiggleBackY = Math.cos(Math.toRadians(normalizedOrientation)) * wiggleBackDistance;

        runBlocking(
                intake.rotate(rotationTarget)
        );

        runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveActions.alignToSample(targetSample.getSamplePosition().position),
                                extendSequence
                        ),
                        grabSequence
                )
        );

//        runBlocking(
//                new SequentialAction(
//                        drive.actionBuilder(drive.pose)
//                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x + wiggleX, drive.pose.position.y - wiggleY), null, new ProfileAccelConstraint(-100, 100))
//                                .afterDisp(wiggleDistance, intake.closeClaw())
//                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x - wiggleBackX, drive.pose.position.y + wiggleBackY), null, new ProfileAccelConstraint(-100, 100))
//                                .build(),
//
//                )
//        );

        transition("putInBasket");
    }

    @State(requiredTime = 2)
    public void park() {
        Action intakeRetract = new ParallelAction(
                intake.retract(),
                intake.wristMiddle()
        );

        switch (strategy) {
            case SPECIMENS:
                // Park in observation zone
                runAsync(intakeRetract);

                runAsync(
                        new ParallelAction(
                                specimenArm.goToHanged(),
                                specimenArm.grabOpen()
                        )
                );

                runAsync(
                        new SequentialAction(
                                specimenArm.grabOpen(),
                                new SleepUntilAction(() -> drive.pose.position.y < -45),
                                specimenArm.gripToIntake(),
                                specimenArm.goToIntake()
                        )
                );

                runBlocking(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(50, startPose.position.y), Math.toRadians(-45), null, new ProfileAccelConstraint(-100, 200))
                                .build()
                );

                requestOpModeStop();
                break;
            case BASKET:
                // Park at the bars
                runAsync(intakeRetract);

                runAsync(
                        new SequentialAction(
                                new SleepUntilAction(() -> drive.pose.position.x > -40),
                                specimenArm.gripToOuttake(),
                                specimenArm.goToPark()
                        )
                );


                if (drive.pose.position.x < -35) {
                    runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(-27, -9), Math.toRadians(0), null, new ProfileAccelConstraint(-100, 200))
                                    .build()
                    );
                }

                requestOpModeStop();
                break;
        }
    }

    @State
    public void resetRobot() {
        runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.pose)
                                .splineToLinearHeading(startPose, 0)
                                .build(),

                        intake.retract(),
                        intake.wristMiddle(),
                        intake.rotate(0),
                        intake.openClaw(),

                        outtake.retract(),
                        outtake.hold(),

                        specimenArm.goToIntake(),
                        specimenArm.gripToOuttake(),
                        specimenArm.grabClose()
                )

        );

        requestOpModeStop();
    }
}