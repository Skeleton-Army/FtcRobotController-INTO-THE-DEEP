package org.firstinspires.ftc.teamcode.opModes;

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

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.autonomous.AutoOpMode;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.general.prompts.OptionPrompt;

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

    Alliance alliance;
    Strategy strategy;

    double startingXPos = 0;
    int collectedSamples = 0;

    @Override
    protected State initialState() {
        return State.HANG_SPECIMEN;
    }

    @Override
    protected void registerStates() {
        addState(State.HANG_SPECIMEN, this::hangSpecimen);
        addState(State.COLLECT_YELLOW_SAMPLE, this::collectYellowSample);
        addState(State.PUT_IN_BASKET, this::putInBasket);
        addState(State.PARK, this::park);
//        addState(State.COLLECT_SPECIMEN, this::pickupSpecimen);
//        addState(State.COLLECT_COLOR_SAMPLES, this::collectColorSamples);
    }

    @Override
    public void setPrompts() {
        choiceMenu.enqueuePrompt(new OptionPrompt("alliance", "SELECT AN ALLIANCE:", "Red", "Blue"));
        choiceMenu.enqueuePrompt(new OptionPrompt("strategy", "SELECT A STRATEGY:", "Specimens", "Basket"));
        choiceMenu.enqueuePrompt(new OptionPrompt("specimens", "SELECT HUMAN PLAYER SPECIMENS:", "0", "1"));
    }

    @Override
    public void init() {
        super.init();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //intake = new Intake(hardwareMap);
        //outtake = new Outtake(hardwareMap);
    }

    @Override
    public void start() {
        // Enable auto bulk reads
        Utilities.setBulkReadsMode(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        // Fetch choices
        String selectedAlliance = choiceMenu.getValueOf("alliance").toString();
        String selectedStrategy = choiceMenu.getValueOf("strategy").toString();
        String selectedSpecimens = choiceMenu.getValueOf("specimens").toString();

        telemetry.addData("Selected Alliance", selectedAlliance);
        telemetry.addData("Selected Strategy", selectedStrategy);
        telemetry.addData("Selected Specimen", selectedSpecimens);

        // Initialize values
        alliance = selectedAlliance.equals("Red") ? Alliance.RED : Alliance.BLUE;
        strategy = selectedStrategy.equals("Specimens") ? Strategy.SPECIMENS : Strategy.BASKET;

        switch (strategy) {
            case SPECIMENS:
                startingXPos = 10;
                break;
            case BASKET:
                startingXPos = -10;
                break;
        }

        // Set starting position
        drive.pose = new Pose2d(startingXPos, -61.5, Math.toRadians(90.00));
    }

    // -------------- States --------------

    private void hangSpecimen() {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                        .splineTo(new Vector2d(startingXPos, -35), Math.PI / 2, null, new ProfileAccelConstraint(-50, 100))
                        .build()
        );

        addConditionalTransition(strategy == Strategy.SPECIMENS, State.COLLECT_COLOR_SAMPLES, State.COLLECT_YELLOW_SAMPLE);
    }

    private void collectYellowSample() {
        collectedSamples++;

        Action extendSequence = new ParallelAction(
                intake.extend(),
                intake.extendWrist(),
                intake.openClaw()
        );

        Action retractSequence = new ParallelAction(
                intake.closeClaw(),
                intake.retractWrist(),
                outtake.hold(),
                new SequentialAction(
                        intake.retract(),
                        intake.openClaw(),
                        new SleepAction(0.5),
                        intake.wristMiddle()
                )
        );

        switch (collectedSamples) {
            case 1:
                // Collect first sample
                Actions.runBlocking(
                        new ParallelAction(
                                drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                        .splineToConstantHeading(new Vector2d(10, -50), Math.PI / 2)
                                        .splineToLinearHeading(new Pose2d(-55, -48, Math.toRadians(75)), Math.PI)
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(1),
                                        extendSequence
                                )
                        )
                );
                Actions.runBlocking(retractSequence);
                break;
            case 2:
                // Collect second sample
                Actions.runBlocking(
                        new ParallelAction(
                                drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                        .splineToLinearHeading(new Pose2d(-60, -48, Math.toRadians(90)), Math.PI)
                                        .build(),
                                extendSequence
                        )
                );
                Actions.runBlocking(retractSequence);
                break;
            case 3:
                // Collect third sample
                Actions.runBlocking(
                        new ParallelAction(
                                drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                        .splineToLinearHeading(new Pose2d(-60, -48, Math.toRadians(105)), Math.PI)
                                        .build(),
                                extendSequence
                        )
                );
                Actions.runBlocking(retractSequence);
                break;
        }

        addTransition(State.PUT_IN_BASKET);
    }

    private void putInBasket() {
        // Put the sample in the basket
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.PI / 2)
                                .build(),
                        outtake.extend()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        outtake.dunk(),
                        new SleepAction(1),
                        new ParallelAction(
                                outtake.retract(),
                                outtake.hold()
                        )
                )
        );

        addConditionalTransition(collectedSamples < 3, State.COLLECT_YELLOW_SAMPLE, State.PARK);
    }

    private void park() {
        switch (strategy) {
            case SPECIMENS:
                // Park in observation zone
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                .splineTo(new Vector2d(50, -60), Math.toRadians(0))
                                .build()
                );
                break;
            case BASKET:
                // Park at bars
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose, alliance == Alliance.BLUE)
                                .splineTo(new Vector2d(-25, -10), Math.toRadians(0))
                                .build()
                );
                break;
        }

        requestOpModeStop();
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