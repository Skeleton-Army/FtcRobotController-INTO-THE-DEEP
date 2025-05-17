package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.wiggleBackDistance;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.wiggleDistance;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
import org.firstinspires.ftc.teamcode.utils.autonomous.AutoOpMode;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.general.prompts.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

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
    public enum State {
        HANG_SPECIMEN(2),
        COLLECT_ADDITIONAL_SAMPLE(4.5),

        COLLECT_COLOR_SAMPLES(),
        COLLECT_SPECIMEN(2),

        COLLECT_YELLOW_SAMPLE(),
        PUT_IN_BASKET(2),

        PARK(2);

        // ------------ Attributes ------------
        private final double requiredTime; // in seconds

        State() {
            this.requiredTime = 0;
        }

        State(double requiredTime) {
            this.requiredTime = requiredTime;
        }

        public double getRequiredTime() {
            return requiredTime;
        }
    }

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
    public void setPrompts() {
        choiceMenu.enqueuePrompt(new OptionPrompt("alliance", "SELECT AN ALLIANCE:", "Red", "Blue"));
        choiceMenu.enqueuePrompt(new OptionPrompt("strategy", "SELECT A STRATEGY:", "Specimens", "Basket"));
        choiceMenu.enqueuePrompt(new OptionPrompt("specimens", "SELECT HUMAN PLAYER SPECIMENS:", "1", "0"));
    }

    @Override
    public void onPromptsSelected() {
        // Fetch choices
        String selectedAlliance = choiceMenu.getValueOf("alliance", "Red").toString();
        String selectedStrategy = choiceMenu.getValueOf("strategy", "Basket").toString();
        String selectedSpecimens = choiceMenu.getValueOf("specimens", "1").toString();

        telemetry.addData("Selected Alliance", selectedAlliance);
        telemetry.addData("Selected Strategy", selectedStrategy);
        telemetry.addData("Selected Specimen", selectedSpecimens);

        // Initialize values
        alliance = selectedAlliance.equals("Red") ? Alliance.RED : Alliance.BLUE;
        strategy = selectedStrategy.equals("Specimens") ? Strategy.SPECIMENS : Strategy.BASKET;
        extraSpecimens = Integer.parseInt(selectedSpecimens);

        switch (strategy) {
            case SPECIMENS:
                startPose = new Pose2d(0, -62.5, Math.toRadians(90.00));
                break;
            case BASKET:
                startPose = new Pose2d(-39, -62.5, Math.toRadians(0));
                break;
        }

//        runAsync(
//                new SequentialAction(
//                        new SleepAction(3),
//                        new InstantAction(() -> camCV.stopStream())
//                )
//        );
    }

    @Override
    protected void registerStates() {
        addState(State.HANG_SPECIMEN, this::hangSpecimen);
        addState(State.COLLECT_YELLOW_SAMPLE, this::collectYellowSample);
        addState(State.PUT_IN_BASKET, this::putInBasket);
        addState(State.COLLECT_ADDITIONAL_SAMPLE, this::sampleFromSubmersible);
        addState(State.PARK, this::park);
        addState(State.COLLECT_SPECIMEN, this::collectSpecimen);
        addState(State.COLLECT_COLOR_SAMPLES, this::collectColorSamples);

        setFallbackState(() -> gamepad1.guide || gamepad2.guide, this::resetRobot);
    }

    @Override
    public void setInitialState() {
        switch (strategy) {
            case SPECIMENS:
                addTransition(State.HANG_SPECIMEN);
                break;
            case BASKET:
                addTransition(State.PUT_IN_BASKET);
                break;
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

        //runAsync(this::outtakeLimitSwitch);
        runAsync(specimenArm::update);
    }

    // -------------- States --------------

    private void hangSpecimen() {
        if (!isEnoughTime(State.HANG_SPECIMEN)) {
            addTransition(State.PARK);
            return;
        }

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
            addTransition(State.COLLECT_COLOR_SAMPLES);
        } else if (hangedSpecimens < (4 + extraSpecimens)) {
            addTransition(State.COLLECT_SPECIMEN);
        } else {
            addTransition(State.PARK);
        }
    }

    private void collectSpecimen() {
        if (!isEnoughTime(State.COLLECT_SPECIMEN)) {
            addTransition(State.PARK);
            return;
        }

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

        addTransition(State.HANG_SPECIMEN);
    }

    private void collectColorSamples() {
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

        addTransition(State.COLLECT_SPECIMEN);
    }

    private void collectYellowSample() {
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
                                        .afterDisp(3.5, wristSequence)
                                        .splineToLinearHeading(new Pose2d(-52, -52.25, Math.toRadians(80)), Math.PI)
                                        .build()
                        )
                );
                break;
            case 2:
                // Collect second sample
                runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .afterDisp(6, wristSequence)
                                        .splineToLinearHeading(new Pose2d(-57.5, -51.25, Math.toRadians(90)), Math.PI)
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
                                        .afterDisp(8, new SequentialAction(
                                                intake.extraOpenClaw(),
                                                wristSequence,
                                                new SleepAction(0.15),
                                                intake.closeClaw()
                                        ))
                                        .splineToLinearHeading(new Pose2d(-55.5, -48.25, Math.toRadians(120)), Math.PI)
                                        .build()
                        )
                );
                break;
        }

        addTransition(State.PUT_IN_BASKET);
    }

    private void putInBasket() {
        if (!isEnoughTime(State.PUT_IN_BASKET)) {
            addTransition(State.PARK);
            return;
        }

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
            double xCompensation = collectedSamples >= 6 ? 1.5 : 0;
            double yCompensation = collectedSamples >= 6 ? 1.5 : 0;
            double angleCompensation = 0;

            if (collectedSamples == 6) angleCompensation = 5;
            if (collectedSamples == 7) angleCompensation = 10;

            runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(200))
                                    .splineToLinearHeading(new Pose2d(-56 + xCompensation, -54.5 - yCompensation, Math.toRadians(45 + angleCompensation)), Math.toRadians(225), null, new ProfileAccelConstraint(-80, 200))
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
            if (collectedSamples != 3) runBlocking(grab);

            // Retract and go to basket
            runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose)
                                    .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(225))
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
            addConditionalTransition(collectedSamples < 3, State.COLLECT_YELLOW_SAMPLE, State.COLLECT_ADDITIONAL_SAMPLE);
        } else {
            addTransition(State.PARK);
        }
    }

    private void sampleFromSubmersible() {
        if (!isEnoughTime(State.COLLECT_ADDITIONAL_SAMPLE)) {
            addTransition(State.PARK);
            return;
        }

        collectedSamples++;

        Action extendSequence = new SequentialAction(
                intake.wristReady(),
                intake.openClaw(),
                new ParallelAction(
                        intake.extend()
//                        new SequentialAction(
//                                new SleepUntilAction(() -> intake.motor.getCurrentPosition() >= 400),
//                                intake.extendWrist()
//                        )
                )
//                new SleepAction(0.1)
        );

        Action grabSequence = new SequentialAction(
                intake.extendWrist(),
                new SleepAction(0.2),
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

        addTransition(State.PUT_IN_BASKET);
    }

    private void park() {
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

    private void resetRobot() {
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