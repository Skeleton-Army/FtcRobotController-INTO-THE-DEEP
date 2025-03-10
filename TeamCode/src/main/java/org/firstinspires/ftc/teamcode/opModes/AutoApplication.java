package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.autonomous.AutoOpMode;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
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
    protected enum State {
        HANG_SPECIMEN,
        COLLECT_ADDITIONAL_SAMPLE,

        COLLECT_COLOR_SAMPLES,
        COLLECT_SPECIMEN,

        COLLECT_YELLOW_SAMPLE,
        PUT_IN_BASKET,

        PARK
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

    boolean gotOne = false;
    boolean didCollectSamples = false;

    @Override
    public void setPrompts() {
//        choiceMenu.enqueuePrompt(new OptionPrompt("alliance", "SELECT AN ALLIANCE:", "Red", "Blue"));
        choiceMenu.enqueuePrompt(new OptionPrompt("strategy", "SELECT A STRATEGY:", "Specimens", "Basket"));
        choiceMenu.enqueuePrompt(new OptionPrompt("specimens", "SELECT HUMAN PLAYER SPECIMENS:", "1", "0"));
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

        camCV = new WebcamCV(hardwareMap, telemetry, drive);
        camCV.configureWebcam(new SampleColor[]{SampleColor.YELLOW});
        //camCV.stopStream(); Maybe?

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);

        outtakeSwitch = hardwareMap.get(DigitalChannel.class, OuttakeConfig.limitSwitchName);

        runBlocking(specimenArm.grabClose());
    }

    @Override
    public void onStart() {
        // Fetch choices
//        String selectedAlliance = choiceMenu.getValueOf("alliance").toString();
        String selectedStrategy = choiceMenu.getValueOf("strategy", "Specimens").toString();
        String selectedSpecimens = choiceMenu.getValueOf("specimens", "1").toString();

        String selectedAlliance = "Red";

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

        // Set starting position
        drive.pose = startPose;

        //runAsync(this::outtakeLimitSwitch);
        runAsync(specimenArm::update);
    }

    // -------------- States --------------

    private void hangSpecimen() {
        hangedSpecimens++;

        int angleCompensation = (hangedSpecimens - 1) * -4;

        runBlocking(
                new ParallelAction(
                        specimenArm.grabClose(),
                        specimenArm.gripToOuttake(),
                        specimenArm.goToOuttake(),

                        drive.actionBuilder(drive.pose)
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
        int angleCompensation = (hangedSpecimens - 1) * -4;

        runBlocking(specimenArm.grabOpen());

        runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(hangedSpecimens == 1 ? 180 : 270))
                                .splineToLinearHeading(new Pose2d(27, -63.2, Math.toRadians(95 + angleCompensation)), Math.toRadians(270), null, new ProfileAccelConstraint(-60, 150))
                                .build(),
                        new SequentialAction(
                                specimenArm.grabOpen(),
                                new SleepAction(0.7),
                                specimenArm.gripToIntake(),
                                specimenArm.goToIntake()
                        )
                )
        );

        runBlocking(
                new SequentialAction(
                        specimenArm.grabClose(),
                        new SleepAction(0.2)
                )
        );

        addTransition(State.HANG_SPECIMEN);
    }

    private void collectColorSamples() {
        didCollectSamples = true;

        runAsync(
                new SequentialAction(
                        specimenArm.grabOpen(),
                        new SleepAction(0.5),
                        specimenArm.gripToIntake(),
                        specimenArm.goToIntake()
                )
        );

        // Grab first sample
        runAsync(
                new SequentialAction(
                        new SleepAction(0.5),
                        intake.wristReady(),
                        intake.extend(0.57)
                )
        );

        runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(48.5, -43, Math.toRadians(95)), 0, null, new ProfileAccelConstraint(-25, 100))
                                .build(),
                        new SequentialAction(
                                intake.openClaw(),
                                intake.extendWrist(),
                                new SleepAction(0.3),
                                intake.closeClaw(),
                                new SleepAction(0.2),
                                intake.wristReady()
                        )
                )
        );

        // Grab second sample
        runBlocking(
                new SequentialAction(
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

                                drive.actionBuilder(drive.pose)
                                        .setTangent(0)
                                        .splineToConstantHeading(new Vector2d(58, -43), 0)
                                        .build()
                        ),
                        new ParallelAction(
                                // Drop sample
                                new SequentialAction(
                                        outtake.extend(0.3),
                                        outtake.dunk(),
                                        new SleepAction(0.8),
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
                                        new SleepAction(0.3),
                                        intake.closeClaw(),
                                        new SleepAction(0.3),
                                        intake.wristReady()
                                )
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
                                outtake.extend(0.3),
                                outtake.dunk(),
                                new SleepAction(0.8)
                        ),

                        new ParallelAction(
                                outtake.hold(),
                                outtake.retract(),

                                drive.actionBuilder(drive.pose)
                                        .setTangent(0)
                                        .splineToLinearHeading(new Pose2d(58, -43, Math.toRadians(58)), 0)
                                        .build(),

                                // Grab sample
                                new SequentialAction(
                                        intake.wristReady(),
                                        intake.openClaw(),
                                        intake.extend(0.77),
                                        new SleepAction(0.3), // Wait for sample to fall out
                                        intake.extendWrist(),
                                        new SleepAction(0.3),
                                        intake.closeClaw(),
                                        new SleepAction(0.2),
                                        intake.wristReady()
                                )
                        ),

                        new ParallelAction(
                                drive.actionBuilder(drive.pose)
                                        .splineToLinearHeading(new Pose2d(58, -43, Math.toRadians(95)), 0)
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
                                outtake.extend(0.3),
                                outtake.dunk(),
                                new SleepAction(0.8),
                                outtake.hold(),
                                outtake.retract()
                        )
                )
        );

        addTransition(State.COLLECT_SPECIMEN);
    }

    private void collectYellowSample() {
        collectedSamples++;

        Action wristSequence = new SequentialAction(
                intake.extendWrist(),
                intake.openClaw(),
                new SleepAction(0.15)
        );

        switch (collectedSamples) {
            case 1:
                // Collect first sample
                runBlocking(
                        new ParallelAction(
                                outtake.retract(),
                                new SequentialAction(
                                        drive.actionBuilder(drive.pose)
                                                .splineToLinearHeading(new Pose2d(-53, -50.5, Math.toRadians(75)), Math.PI)
                                                .build(),
                                        wristSequence
                                )
                        )
                );
                break;
            case 2:
                // Collect second sample
                runBlocking(
                        new ParallelAction(
                                outtake.retract(),
                                new SequentialAction(
                                        drive.actionBuilder(drive.pose)
                                                .splineToLinearHeading(new Pose2d(-56, -50.5, Math.toRadians(90)), Math.PI)
                                                .build(),
                                        wristSequence
                                )
                        )
                );
                break;
            case 3:
                // Collect third sample
                runBlocking(
                        new ParallelAction(
                                outtake.retract(),
                                new SequentialAction(
                                        drive.actionBuilder(drive.pose)
                                                .splineToLinearHeading(new Pose2d(-56, -48, Math.toRadians(115)), Math.PI)
                                                .build(),
                                        wristSequence
                                )
                        )
                );
                break;
        }

        addTransition(State.PUT_IN_BASKET);
    }

    private void putInBasket() {
        Action grab = new SequentialAction(
                intake.closeClaw(),
                new SleepAction(0.1)
        );

        Action intakeRetract = new ParallelAction(
                intake.retractWrist(),
                outtake.hold(),
                new SequentialAction(
                        new ParallelAction(
                                intake.retract(collectedSamples == 4 ? 0.7 : 1),
                                new SleepAction(0.4)
                        ),
                        intake.openClaw(),
                        intake.wristReady(),
                        new SleepAction(0.2)
                )
        );

        Action dunkSample = new SequentialAction(
                outtake.extend(),
                outtake.dunk(),
                new SleepAction(0.6),
                outtake.hold()
        );

        if (collectedSamples == 0) {
            runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setTangent(Math.PI / 2)
                            .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.PI)
                            .build()
            );
        }
        else if (collectedSamples == 4) {
            runBlocking(grab);

            runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.PI)
                                    .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(225))
                                    .build(),
                            new SequentialAction(
                                    intakeRetract,
                                    dunkSample
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
                                    .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(225))
                                    .build(),
                            intakeRetract
                    )
            );
        }

        // Put the sample in the basket
        if (collectedSamples != 4) {
            runAsync(
                    new ParallelAction(
                            intake.extend(0.9),
                            intake.wristReady()
                    )
            );
        }

        runBlocking(
                dunkSample
        );

        if (!gotOne) {
            addConditionalTransition(collectedSamples < 3, State.COLLECT_YELLOW_SAMPLE, State.COLLECT_ADDITIONAL_SAMPLE);
        } else {
            addTransition(State.PARK);
        }
    }

    private void sampleFromSubmersible() {
        gotOne = true;

        Action wristSequence = new SequentialAction(
                intake.extendWrist(),
                intake.openClaw(),
                new SleepAction(0.5)
        );

        runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(-32, -10), Math.toRadians(0))
                                .build(),
                        outtake.retract()
                )
        );
        camCV.resetSampleList();
        runBlocking(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        return !camCV.lookForSamples();
                    }
                }
        );

        Vector2d samplePos = camCV.getBestSamplePos(new Vector2d(-5, 0)).position;
        // TODO: Add some sort of validation For example if (bad == yes): don't.

        int m = samplePos.y > 0 ? -1 : 1;
        runBlocking(
                new SequentialAction(
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            try {
                                double heading = drive.pose.heading.toDouble();
                                Vector2d offset = new Vector2d(CameraConfig.pickupSampleOffsetY* Math.cos(heading) -
                                        CameraConfig.pickupSampleOffsetX * m * Math.sin(heading),
                                        CameraConfig.pickupSampleOffsetY * Math.sin(heading) +
                                                CameraConfig.pickupSampleOffsetX * m * Math.cos(heading));
                                Vector2d sampleAlignment = samplePos.minus(offset);

                                telemetryPacket.addLine(samplePos.toString());

                                runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .splineToConstantHeading(sampleAlignment, 0) // TODO: shouldn't tangent be the heading, like in alignToSample?
                                                .build()
                                );
                            }

                            catch (Exception e) {
                                telemetryPacket.addLine(e.toString());
                            }

                            return false;
                        }
                    },
                    wristSequence
                )
        );

        collectedSamples++;
        addTransition(State.PUT_IN_BASKET);
    }
    private void park() {
        Action intakeRetract = new ParallelAction(
                intake.retract(),
                intake.wristMiddle(),
                outtake.retract()
        );

        switch (strategy) {
            case SPECIMENS:
                // Park in observation zone
                runBlocking(
                        new ParallelAction(
                                new SequentialAction(
                                        specimenArm.grabOpen(),
                                        new SleepAction(0.7),
                                        specimenArm.gripToIntake(),
                                        specimenArm.goToIntake()
                                ),
                                drive.actionBuilder(drive.pose)
                                        .setTangent(Math.toRadians(-45))
                                        .splineTo(new Vector2d(50, startPose.position.y), Math.toRadians(-45), null, new ProfileAccelConstraint(-60, 150))
                                        .build(),
                                intakeRetract
                        )
                );

                requestOpModeStop();
                break;
            case BASKET:
                // Park at bars
                runBlocking(
                        new ParallelAction(
                                drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(-27, -10), Math.toRadians(0))
                                    .build(),
                                intakeRetract
                        )
                );

                runBlocking(
                        new SequentialAction(
                                specimenArm.gripToIntake(),
                                specimenArm.goToPark()
                        )
                );

                requestOpModeStop();
                break;
        }
    }

    private void outtakeLimitSwitch() {
        if (Utilities.isPressed(!outtakeSwitch.getState())) {
            outtake.resetMotor();
        }
    }
}