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
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.autonomous.AutoOpMode;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
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

    Pose2d startPose;
    WebcamCV camCV;
    int collectedSamples = 0;

    @Override
    protected void registerStates() {
        addState(State.HANG_SPECIMEN, this::hangSpecimen);
        addState(State.COLLECT_YELLOW_SAMPLE, this::collectYellowSample);
        addState(State.PUT_IN_BASKET, this::putInBasket);
        addState(State.COLLECT_ADDITIONAL_SAMPLE, this::sampleFromSubmersible);
        addState(State.PARK, this::park);
//        addState(State.COLLECT_SPECIMEN, this::pickupSpecimen);
//        addState(State.COLLECT_COLOR_SAMPLES, this::collectColorSamples);
    }

    @Override
    public void setPrompts() {
//        choiceMenu.enqueuePrompt(new OptionPrompt("alliance", "SELECT AN ALLIANCE:", "Red", "Blue"));
        choiceMenu.enqueuePrompt(new OptionPrompt("strategy", "SELECT A STRATEGY:", "Specimens", "Basket"));
//        choiceMenu.enqueuePrompt(new OptionPrompt("specimens", "SELECT HUMAN PLAYER SPECIMENS:", "0", "1"));
    }

    @Override
    public void init() {
        super.init();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        camCV = new WebcamCV(hardwareMap, telemetry, drive);
        camCV.configureWebcam(SampleColor.YELLOW);
        //camCV.stopStream(); Maybe?
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);
    }

    @Override
    public void start() {
        // Enable auto bulk reads
        Utilities.setBulkReadsMode(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        // Fetch choices
//        String selectedAlliance = choiceMenu.getValueOf("alliance").toString();
        String selectedStrategy = choiceMenu.getValueOf("strategy").toString();
//        String selectedSpecimens = choiceMenu.getValueOf("specimens").toString();

        String selectedAlliance = "Red";
        String selectedSpecimens = "0";

        telemetry.addData("Selected Alliance", selectedAlliance);
        telemetry.addData("Selected Strategy", selectedStrategy);
        telemetry.addData("Selected Specimen", selectedSpecimens);

        // Initialize values
        alliance = selectedAlliance.equals("Red") ? Alliance.RED : Alliance.BLUE;
        strategy = selectedStrategy.equals("Specimens") ? Strategy.SPECIMENS : Strategy.BASKET;

        switch (strategy) {
            case SPECIMENS:
                startPose = new Pose2d(25, -62.5, Math.toRadians(90.00));
                break;
            case BASKET:
                startPose = new Pose2d(-39, -62.5, Math.toRadians(0));
                break;
        }

        // Set starting position
        drive.pose = startPose;

        setInitialState();
    }

    @Override
    public void setInitialState() {
        switch (strategy) {
            case SPECIMENS:
                addTransition(State.PARK);
                break;
            case BASKET:
                addTransition(State.PUT_IN_BASKET);
                break;
        }
    }

    // -------------- States --------------

    private void hangSpecimen() {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                        .splineTo(new Vector2d(startPose.position.x, -40), Math.PI / 2, null, new ProfileAccelConstraint(-50, 75))
                        .setTangent(Math.toRadians(275))
                        .build()
        );

        addConditionalTransition(strategy == Strategy.SPECIMENS, State.COLLECT_COLOR_SAMPLES, State.COLLECT_YELLOW_SAMPLE);
    }

    private void collectYellowSample() {
        collectedSamples++;

        Action wristSequence = new SequentialAction(
                intake.extendWrist(),
                intake.openClaw(),
                new SleepAction(0.5)
        );

        switch (collectedSamples) {
            case 1:
                // Collect first sample
                Actions.runBlocking(
                        new ParallelAction(
                                outtake.retract(),
                                new SequentialAction(
                                        drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                                .splineToLinearHeading(new Pose2d(-53, -50.5, Math.toRadians(75)), Math.PI)
                                                .build(),
                                        wristSequence
                                )
                        )
                );
                break;
            case 2:
                // Collect second sample
                Actions.runBlocking(
                        new ParallelAction(
                                outtake.retract(),
                                new SequentialAction(
                                        drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                                .splineToLinearHeading(new Pose2d(-56, -50.5, Math.toRadians(90)), Math.PI)
                                                .build(),
                                        wristSequence
                                )
                        )
                );
                break;
            case 3:
                // Collect third sample
                Actions.runBlocking(
                        new ParallelAction(
                                outtake.retract(),
                                new SequentialAction(
                                        drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
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
                new SleepAction(0.2)
        );

        Action intakeRetract = new SequentialAction(
                intake.retractWrist(),
                outtake.hold(),
                intake.retract(),
                intake.openClaw(),
                new SleepAction(0.2),
                intake.wristMiddle(),
                new SleepAction(0.2)
        );

        Action dunkSample = new SequentialAction(
                outtake.extend(),
                outtake.dunk(),
                new SleepAction(1.2),
                outtake.hold()
        );

        if (collectedSamples == 0) {
            Actions.runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                    .setTangent(Math.PI / 2)
                                    .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.PI)
                                    .build(),
                            intake.extend()
                    )
            );
        }
        else if (collectedSamples == 4) {
            Actions.runBlocking(grab);

            Actions.runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                    .setTangent(Math.PI)
                                    .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.PI / 2)
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
            Actions.runBlocking(grab);

            // Retract and go to basket
            Actions.runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                    .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.PI / 2)
                                    .build(),
                            intakeRetract
                    )
            );
        }

        // Put the sample in the basket
        Actions.runBlocking(
                new ParallelAction(
                        dunkSample,
                        intake.extend()
                )
        );

        addConditionalTransition(collectedSamples < 3, State.COLLECT_YELLOW_SAMPLE, State.COLLECT_ADDITIONAL_SAMPLE);
    }

    private void sampleFromSubmersible() {
        Action wristSequence = new SequentialAction(
                intake.extendWrist(),
                intake.openClaw(),
                new SleepAction(0.5)
        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                .splineTo(new Vector2d(-32, -10), Math.toRadians(0))
                                .build(),
                        outtake.retract()
                )
        );
        camCV.resetSampleList();
        Actions.runBlocking(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        return !camCV.lookForSamples();
                    }
                }
        );

        Vector2d samplePos = camCV.getBestSamplePos(new Vector2d(-2, -2));
        // TODO: Add some sort of validation For example if (bad == yes): don't.

        Actions.runBlocking(
                new SequentialAction(
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            try {
                                double heading = drive.pose.heading.toDouble();
                                Vector2d offset = new Vector2d(CameraConfig.pickupSampleOffsetY* Math.cos(heading) -
                                        CameraConfig.pickupSampleOffsetX * Math.sin(heading),
                                        CameraConfig.pickupSampleOffsetY * Math.sin(heading) +
                                                CameraConfig.pickupSampleOffsetX * Math.cos(heading));
                                Vector2d sampleAlignment = samplePos.minus(offset);

                                telemetryPacket.addLine(samplePos.toString());

                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .splineToConstantHeading(sampleAlignment, 0)
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
                Actions.runBlocking(
                        new ParallelAction(
                                drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                        .setTangent(0)
                                        .splineToConstantHeading(new Vector2d(50, startPose.position.y), 0)
                                        .build(),
                                intakeRetract
                        )
                );
                break;
            case BASKET:
//                specimenArm.setTarget(SpecimenArmConfig.outtakePosition);
//                specimenArm.update();
                specimenArm.setPower(-1);

                // Park at bars
                Actions.runBlocking(
                        new ParallelAction(
                                drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                    .splineTo(new Vector2d(-27, -10), Math.toRadians(0))
                                    .build(),
                                intakeRetract
                        )
                );
                break;
        }

//        requestOpModeStop();
    }

//    private void collectColorSamples() {
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
//                        .splineToSplineHeading(new Pose2d(17.07, -49.05, Math.toRadians(90)), Math.toRadians(-14.83))
//                        .splineToLinearHeading(new Pose2d(48, -43, Math.toRadians(90)), Math.toRadians(67.62))
//                        .build()
//        );
//
//        setState(State.COLLECT_SPECIMEN);
//    }
//
//    private void pickupSpecimen() {
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
//                        .splineToLinearHeading(new Pose2d(35, -55, Math.toRadians(-90)), Math.toRadians(180))
//                        .build()
//        );
//
//        setState(State.HANG_SPECIMEN);
//    }
}