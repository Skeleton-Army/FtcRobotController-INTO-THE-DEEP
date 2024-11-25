package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

import java.util.List;

@Autonomous(name = "Autonomous App", group = "SA_FTC", preselectTeleOp = "Teleop App")
public class AutoApplication extends AutoOpMode {
    public enum State {
        HANG_SPECIMEN,
        PICKUP_SPECIMEN
    }

    Intake intake;
    Outtake outtake;

    String alliance;

    @Override
    protected State initialState() {
        return State.HANG_SPECIMEN;
    }

    @Override
    protected void registerStates() {
        addState(State.HANG_SPECIMEN, this::hangSpecimen);
        addState(State.PICKUP_SPECIMEN, this::pickupSpecimen);
    }

    @Override
    public void setPrompts() {
        choiceMenu.enqueuePrompt(new OptionPrompt("alliance", "SELECT AN ALLIANCE:", "Red", "Blue"));
        choiceMenu.enqueuePrompt(new OptionPrompt("position", "SELECT THE STARTING POSITION:", "Observation Zone Side", "Basket Side"));
        choiceMenu.enqueuePrompt(new OptionPrompt("strategy", "SELECT A STRATEGY:", "Specimens", "Yellow Samples"));
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
        super.start();

        // Enable auto bulk reads
        Utilities.setBulkReadsMode(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        alliance = choiceMenu.getValueOf("alliance").toString();
        String position = choiceMenu.getValueOf("position").toString();
        String strategy = choiceMenu.getValueOf("strategy").toString();
        String specimens = choiceMenu.getValueOf("specimens").toString();

        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Position", position);
        telemetry.addData("Selected Strategy", strategy);
        telemetry.addData("Selected Specimen", specimens);

        drive.pose = new Pose2d(10, -61.5, Math.toRadians(90.00));
    }

    private void hangSpecimen() {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose, alliance.equals("Blue"))
                        .splineTo(new Vector2d(10, -35), Math.PI / 2, null, new ProfileAccelConstraint(-1000, 100))
                        .build()
        );

        setState(State.PICKUP_SPECIMEN);
    }

    private void pickupSpecimen() {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose, alliance.equals("Blue"))
                        .splineToSplineHeading(new Pose2d(17.07, -49.05, Math.toRadians(90)), Math.toRadians(-14.83))
                        .splineToLinearHeading(new Pose2d(41.94, -39.98, Math.toRadians(60)), Math.toRadians(67.62))
                        .splineTo(new Vector2d(46.07, -45.68), Math.toRadians(-70.35))
                        .build()
        );

        setState(State.HANG_SPECIMEN);
    }
}