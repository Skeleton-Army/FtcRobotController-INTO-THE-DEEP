package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.actions.FollowPath;
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
import org.firstinspires.ftc.teamcode.utils.autonomous.AutoOpMode;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.general.prompts.OptionPrompt;
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

    Follower follower;

    Intake intake;
    Outtake outtake;
    SpecimenArm specimenArm;

    Drive driveActions;

    Alliance alliance;
    Strategy strategy;
    int extraSpecimens;

    Pose startPose;
    WebcamCV camCV;

    DigitalChannel outtakeSwitch;

    int collectedSamples = 0;
    int hangedSpecimens = 0;

    boolean didCollectSamples = false;

    private final Pose basketPose = new Pose(128.00, 16.00, Math.toRadians(135.00));
    private final Pose preloadCP = new Pose(125.28, 26.85);
    private final Pose submersibleCP1 = new Pose(83.52, 21.48);
    private final Pose submersibleCP2 = new Pose(107.95, 36.04);

    private final Pose toSubmersiblePose = new Pose(77.00, 40.00, Math.toRadians(90.00));
    private final Pose toSubmersibleCP = new Pose(91.66, 20.62);

    private final Pose submersibleParkPose = new Pose(81.00, 45.00, Math.toRadians(90.00));
    private final Pose observationParkPose = new Pose(134.50, 122.00, Math.toRadians(-120.00));

    private final Pose yellowSample1Pose = new Pose(124.25, 20.00, Math.toRadians(170.00));
    private final Pose yellowSample2Pose = new Pose(123.25, 14.75, Math.toRadians(180.00));
    private final Pose yellowSample3Pose = new Pose(120.25, 16.50, Math.toRadians(-150.00));

    private final Pose colorSample1Pose = new Pose(114.00, 120.50, Math.toRadians(180.00));
    private final Pose colorSample2Pose = new Pose(112.50, 130.00, Math.toRadians(180.00));
    private final Pose colorSample3Pose = new Pose(113.50, 128.00, Math.toRadians(150.00));
    private final Pose colorSample3DropPose = new Pose(111.00, 129.50, Math.toRadians(180.00));

    private final Pose hangSpecimenPose = new Pose(104.50, 72.00, Math.toRadians(180.00));
    private final Pose hangSpecimenCP = new Pose(133.77, 71.91);

    private final Pose collectSpecimenPose = new Pose(135.20, 99.00, Math.toRadians(180.00));

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
                startPose = new Pose(134.50, 72.00, Math.toRadians(-180.00));
                break;
            case BASKET:
                startPose = new Pose(134.50, 33.00, Math.toRadians(90.00));
                break;
        }
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
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

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
        follower.setStartingPose(startPose);

        // Configure webcam CV
        camCV = new WebcamCV(hardwareMap, telemetry, follower);
        camCV.configureWebcam(new SampleColor[] { SampleColor.YELLOW, alliance == Alliance.RED ? SampleColor.RED : SampleColor.BLUE });

        driveActions = new Drive(follower, camCV, telemetry);

        //runAsync(this::outtakeLimitSwitch);
        runAsync(specimenArm::update);
        runAsync(() -> follower.update());
    }

    // -------------- States --------------

    private void hangSpecimen() {
        if (!isEnoughTime(State.HANG_SPECIMEN)) {
            addTransition(State.PARK);
            return;
        }

        hangedSpecimens++;

        runBlocking(
                new ParallelAction(
                        specimenArm.grabClose(),
                        specimenArm.gripToOuttake(),
                        specimenArm.goToOuttake(),

                        new FollowPath(follower, follower.pathBuilder()
                                .addPath(new BezierCurve(follower.getPose(), hangSpecimenCP, hangSpecimenPose))
                                .setConstantHeadingInterpolation(hangSpecimenPose.getHeading())
                                .build()
                        )
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
                        new FollowPath(follower, follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), collectSpecimenPose))
                                .setConstantHeadingInterpolation(collectSpecimenPose.getHeading())
                                .build()
                        ),

                        new SequentialAction(
                                specimenArm.grabOpen(),
                                new SleepUntilAction(() -> follower.getPose().getX() > 117),
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
                        new SleepUntilAction(() -> follower.getPose().getX() > 122),
                        specimenArm.gripToIntake(),
                        specimenArm.goToIntake()
                )
        );

        // Grab first sample
        runAsync(
                new SequentialAction(
                        new SleepUntilAction(() -> follower.getPose().getX() > 114),
                        intake.wristReady(),
                        intake.extend(0.52)
                )
        );

        runBlocking(
                new SequentialAction(
                        new FollowPath(follower, follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), colorSample1Pose))
                                .setConstantHeadingInterpolation(colorSample1Pose.getHeading())
                                .build()
                        ),

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

                        new FollowPath(follower, follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), colorSample2Pose))
                                .setConstantHeadingInterpolation(colorSample2Pose.getHeading())
                                .build()
                        )
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

                                new FollowPath(follower, follower.pathBuilder()
                                        .addPath(new BezierLine(follower.getPose(), colorSample3Pose))
                                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), colorSample3Pose.getHeading())
                                        .build()
                                ),

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
                                new FollowPath(follower, follower.pathBuilder()
                                        .addPath(new BezierLine(follower.getPose(), colorSample3DropPose))
                                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), colorSample3DropPose.getHeading())
                                        .build()
                                ),

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
                                new FollowPath(follower, follower.pathBuilder()
                                        .addPath(new BezierLine(follower.getPose(), yellowSample1Pose))
                                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), yellowSample1Pose.getHeading())
                                        .build()
                                ),
                                wristSequence
                        )
                );
                break;
            case 2:
                // Collect second sample
                runBlocking(
                        new SequentialAction(
                                new FollowPath(follower, follower.pathBuilder()
                                        .addPath(new BezierLine(follower.getPose(), yellowSample2Pose))
                                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), yellowSample2Pose.getHeading())
                                        .build()
                                ),
                                wristSequence
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
                                new FollowPath(follower, follower.pathBuilder()
                                        .addPath(new BezierLine(follower.getPose(), yellowSample3Pose))
                                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), yellowSample3Pose.getHeading())
                                        .build()
                                ),

                                intake.extraOpenClaw(),
                                wristSequence,
                                new SleepAction(0.15),
                                intake.closeClaw()
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
                            new FollowPath(follower, follower.pathBuilder()
                                    .addPath(new BezierCurve(follower.getPose(), preloadCP, basketPose))
                                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), basketPose.getHeading())
                                    .build()
                            ),
                            dunkSequence
                    )
            );
        }
        else if (collectedSamples >= 4) {
            runBlocking(
                    new ParallelAction(
                            new FollowPath(follower, follower.pathBuilder()
                                    .addPath(new BezierCurve(follower.getPose(), submersibleCP1, submersibleCP2, basketPose))
                                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), basketPose.getHeading())
                                    .build()
                            ),
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
                            new FollowPath(follower, follower.pathBuilder()
                                    .addPath(new BezierLine(follower.getPose(), basketPose))
                                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), basketPose.getHeading())
                                    .build()
                            ),
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

        addConditionalTransition(collectedSamples < 3, State.COLLECT_YELLOW_SAMPLE, State.COLLECT_ADDITIONAL_SAMPLE);
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

        runBlocking(
                new FollowPath(follower, follower.pathBuilder()
                        .addPath(new BezierCurve(follower.getPose(), toSubmersibleCP, toSubmersiblePose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), toSubmersiblePose.getHeading())
                        .build()
                )
        );

        runBlocking(
                new SleepAction(0.4)
        );

        Pose lower = new Pose(82.00, 64.00);
        Pose upper = new Pose(64.00, 77.00);

        camCV.resetSampleList();

        // TODO: Add some sort of validation For example if (bad == yes): don't. This is here because funny. There will never be any validation, deal with it.

        // Grab sample
        runBlocking(
                new SleepUntilAction(() -> camCV.lookForSamples())
        );

        Pose bestSamplePos = new Pose(follower.getPose().getX() - CameraConfig.pickupSampleOffsetX, 69);

        Sample targetSample = camCV.getBestSampleInRange(bestSamplePos, lower, upper);

        if (targetSample == null) targetSample = camCV.getBestSample(bestSamplePos);

        double orientation = -targetSample.orientation;
        double normalizedOrientation = (90 - Math.abs(orientation)) * Math.signum(orientation);
        double rotationTarget = normalizedOrientation / 90;

        runBlocking(
                intake.rotate(rotationTarget)
        );

        runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveActions.alignToSample(targetSample.getSamplePosition()),
                                extendSequence
                        ),
                        grabSequence
                )
        );

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
                                new SleepUntilAction(() -> follower.getPose().getX() > 117),
                                specimenArm.gripToIntake(),
                                specimenArm.goToIntake()
                        )
                );

                runBlocking(
                        new FollowPath(follower, follower.pathBuilder()
                                .addPath(new BezierCurve(follower.getPose(), observationParkPose))
                                .setTangentHeadingInterpolation()
                                .setReversed(true)
                                .build()
                        )
                );

                requestOpModeStop();
                break;
            case BASKET:
                // Park at the bars
                runAsync(intakeRetract);

                runAsync(
                        new SequentialAction(
                                new SleepUntilAction(() -> follower.getPose().getY() > 32),
                                specimenArm.gripToOuttake(),
                                specimenArm.goToPark()
                        )
                );


                if (follower.getPose().getY() < 37) {
                    runBlocking(
                            new FollowPath(follower, follower.pathBuilder()
                                    .addPath(new BezierCurve(follower.getPose(), toSubmersibleCP, submersibleParkPose))
                                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), submersibleParkPose.getHeading())
                                    .build()
                            )
                    );
                }

                requestOpModeStop();
                break;
        }
    }

    private void resetRobot() {
        runBlocking(
                new ParallelAction(
                        new FollowPath(follower, follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), startPose))
                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), startPose.getHeading())
                                .build()
                        ),

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