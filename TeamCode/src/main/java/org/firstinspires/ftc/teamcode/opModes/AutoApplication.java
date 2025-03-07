package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
import org.firstinspires.ftc.teamcode.utils.actions.AlignToSample;
import org.firstinspires.ftc.teamcode.utils.autonomous.AutoOpMode;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.general.prompts.OptionPrompt;
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
        COLLECT_ADDITIONAL_SAMPLE(3),

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
        // Configure webcam CV
        camCV = new WebcamCV(hardwareMap, telemetry, drive);
        camCV.configureWebcam(new SampleColor[] { SampleColor.YELLOW, alliance == Alliance.RED ? SampleColor.RED : SampleColor.BLUE });

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
    }

    @Override
    public void onStart() {
        // Set starting position
        drive.pose = startPose;

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

                        drive.smartActionBuilder().getBuilder()
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(startPose.position.x, -37, Math.toRadians(95 + angleCompensation)), Math.PI / 2, null, new ProfileAccelConstraint(-1000000, hangedSpecimens == 1 ? 100 : 150))
                                .splineToLinearHeading(new Pose2d(startPose.position.x,  hangedSpecimens == 5 ? -30 : -32.5, Math.toRadians(95 + angleCompensation)), Math.PI / 2, null, new ProfileAccelConstraint(-60, hangedSpecimens == 1 ? 100 : 150))
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
                        drive.smartActionBuilder().getBuilder()
                                .setTangent(Math.toRadians(hangedSpecimens == 1 ? 180 : 270))
                                .splineToLinearHeading(new Pose2d(27, -63.2, Math.toRadians(95 + angleCompensation)), Math.toRadians(270), null, new ProfileAccelConstraint(-60, 150))
                                .build(),
                        new SequentialAction(
                                specimenArm.grabOpen(),
                                new SleepUntilAction(() -> drive.pose.position.x < -42),
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
                        new SleepUntilAction(() -> drive.pose.position.x < -42),
                        specimenArm.gripToIntake(),
                        specimenArm.goToIntake()
                )
        );

        // Grab first sample
        runAsync(
                new SequentialAction(
                        new SleepUntilAction(() -> drive.pose.position.x < -42),
                        intake.wristReady(),
                        intake.extend(0.57)
                )
        );

        runBlocking(
                new SequentialAction(
                        drive.smartActionBuilder().getBuilder()
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(48.5, -43, Math.toRadians(95)), 0, null, new ProfileAccelConstraint(-25, 100))
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
                                        new SleepAction(0.4)
                                ),
                                intake.openClaw(),
                                intake.wristReady(),
                                new SleepAction(0.2)
                        ),

                        drive.smartActionBuilder().getBuilder()
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(58, -43), 0)
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
                                        new SequentialAction(
                                                new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -250),
                                                outtake.bucketReady()
                                        )
                                ),
                                new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -300),
                                outtake.dunk(),
                                new SleepAction(0.25),
                                outtake.hold(),
                                outtake.retract()
                        ),

                        // Grab sample
                        new SequentialAction(
                                intake.wristReady(),
                                intake.openClaw(),
                                intake.extend(0.57),
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
                                        new SleepAction(0.4)
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
                                        new SequentialAction(
                                                new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -250),
                                                outtake.bucketReady()
                                        )
                                ),
                                new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -300),
                                outtake.dunk(),
                                new SleepAction(0.25)
                        ),

                        new ParallelAction(
                                outtake.hold(),
                                outtake.retract(),

                                drive.smartActionBuilder().getBuilder()
                                        .setTangent(0)
                                        .splineToLinearHeading(new Pose2d(58, -41, Math.toRadians(55)), 0)
                                        .build(),

                                // Grab sample
                                new SequentialAction(
                                        intake.wristReady(),
                                        intake.openClaw(),
                                        intake.extend(0.73),
                                        intake.extendWrist(),
                                        new SleepAction(0.2),
                                        intake.closeClaw(),
                                        new SleepAction(0.1),
                                        intake.wristReady()
                                )
                        ),

                        new ParallelAction(
                                drive.smartActionBuilder().getBuilder()
                                        .splineToLinearHeading(new Pose2d(58, -41, Math.toRadians(95)), 0)
                                        .build(),

                                // Put in basket
                                intake.retractWrist(),
                                outtake.hold(),
                                new SequentialAction(
                                        new ParallelAction(
                                                intake.retract(),
                                                new SleepAction(0.4)
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
                                        new SequentialAction(
                                                new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -250),
                                                outtake.bucketReady()
                                        )
                                ),
                                new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -300),
                                outtake.dunk()
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
                intake.extendWrist(),
                intake.openClaw(),
                new SleepAction(0.2)
        );

        switch (collectedSamples) {
            case 1:
                // Collect first sample
                runBlocking(
                        new SequentialAction(
                                drive.smartActionBuilder().getBuilder()
                                        .splineToLinearHeading(new Pose2d(-52, -51, Math.toRadians(80)), Math.PI)
                                        .build(),
                                wristSequence
                        )
                );
                break;
            case 2:
                // Collect second sample
                runBlocking(
                        new SequentialAction(
                                drive.smartActionBuilder().getBuilder()
                                        .splineToLinearHeading(new Pose2d(-57, -50.5, Math.toRadians(90)), Math.PI)
                                        .build(),
                                wristSequence
                        )
                );
                break;
            case 3:
                // Collect third sample
                runBlocking(
                        new SequentialAction(
                                drive.smartActionBuilder().getBuilder()
                                        .splineToLinearHeading(new Pose2d(-57.75, -48.5, Math.toRadians(112)), Math.PI)
                                        .build(),
                                wristSequence
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
                new SleepAction(0.2)
        );

        Action intakeRetract = new SequentialAction(
                outtake.hold(),
                intake.retractWrist(),
                new ParallelAction(
                        intake.retract(collectedSamples >= 4 ? 0.7 : 1),
                        new SleepAction(0.4)
                ),
                intake.openClaw(),
                intake.wristReady(),
                new SleepAction(0.2)
        );

        Action extendOuttake = new ParallelAction(
                outtake.bucketMiddle(),
                outtake.extend(),
                new SequentialAction(
                        new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -500),
                        outtake.bucketReady()
                )
        );

        Action dunk = new SequentialAction(
                new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -900),
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
                            drive.smartActionBuilder().getBuilder()
                                    .setTangent(Math.PI / 2)
                                    .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(225), null, new ProfileAccelConstraint(-100, 200))
                                    .build(),
                            dunkSequence
                    )
            );
        }
        else if (collectedSamples >= 4) {
            runBlocking(
                    new ParallelAction(
                            drive.smartActionBuilder().getBuilder()
                                    .setTangent(Math.toRadians(200))
                                    .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)), Math.toRadians(225), null, new ProfileAccelConstraint(-100, 300))
                                    .build(),
                            new SequentialAction(
                                    intakeRetract,
                                    extendOuttake,
                                    dunk
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
                            drive.smartActionBuilder().getBuilder()
                                    .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(225))
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

        Action grabSequence = new SequentialAction(
                intake.wristReady(),
                intake.extend(),
                intake.extendWrist(),
                intake.openClaw(),
                new SleepAction(0.2),
                intake.closeClaw(),
                new SleepAction(0.2)
        );

        runBlocking(
                drive.smartActionBuilder().getBuilder()
                        .splineTo(new Vector2d(-32, -10), Math.toRadians(0), null, new ProfileAccelConstraint(-100, 200))
                        .build()
        );

        // TODO: Add some sort of validation For example if (bad == yes): don't. This is here because funny. There will never be any validation, deal with it.

        // Find first sample position
        findNewSamples();
        Vector2d firstSamplePos = camCV.getBestSamplePos(new Vector2d(-5, -7)).position;

        // First trajectory
        runBlocking(new AlignToSample(drive, firstSamplePos));

        // Find second sample position
        findNewSamples();
        Vector2d secondSamplePos = camCV.getBestSamplePos(firstSamplePos).position;

        // Grab sample
        runBlocking(
                new ParallelAction(
                        grabSequence,
                        new AlignToSample(drive, secondSamplePos)
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
                        new SequentialAction(
                                specimenArm.grabOpen(),
                                new SleepUntilAction(() -> drive.pose.position.x < -42),
                                specimenArm.gripToIntake(),
                                specimenArm.goToIntake()
                        )
                );

                runBlocking(
                        drive.smartActionBuilder().getBuilder()
                                .setTangent(Math.toRadians(-45))
                                .splineTo(new Vector2d(50, startPose.position.y), Math.toRadians(-45), null, new ProfileAccelConstraint(-60, 150))
                                .build()
                );

                requestOpModeStop();
                break;
            case BASKET:
                // Park at the bars
                runAsync(intakeRetract);

                runAsync(
                        new SequentialAction(
                                new SleepUntilAction(() -> drive.pose.position.x > -35),
                                specimenArm.gripToOuttake(),
                                specimenArm.goToPark()
                        )
                );


                if (drive.pose.position.x < -35) {
                    runBlocking(
                            drive.smartActionBuilder().getBuilder()
                                    .splineTo(new Vector2d(-27, -9), Math.toRadians(0))
                                    .build()
                    );
                }

                requestOpModeStop();
                break;
        }
    }

    private void findNewSamples() {
        camCV.resetSampleList();

        runBlocking(
                new SleepUntilAction(() -> camCV.lookForSamples())
        );
    }
}