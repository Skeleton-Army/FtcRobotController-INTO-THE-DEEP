package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.general.ChoiceMenu;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.prompts.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.general.prompts.ValuePrompt;

import java.util.List;

@Autonomous(name = "Autonomous App", group = "SA_FTC", preselectTeleOp = "Teleop App")
public class AutoApplication extends OpMode {
    public enum State {
        HANG_SPECIMEN,
        PICKUP_SPECIMEN
    };

    State state = State.HANG_SPECIMEN;

    ChoiceMenu choiceMenu;

    MecanumDrive drive;
    Intake intake;
    Outtake outtake;

    String alliance;

    private void setPrompts() {
        choiceMenu.enqueuePrompt(new OptionPrompt("alliance", "SELECT AN ALLIANCE:", "Red", "Blue"));
        choiceMenu.enqueuePrompt(new OptionPrompt("position", "SELECT THE STARTING POSITION:", "Observation Zone Side", "Basket Side"));
        choiceMenu.enqueuePrompt(new OptionPrompt("strategy", "SELECT A STRATEGY:", "Specimens", "Yellow Samples"));
        choiceMenu.enqueuePrompt(new OptionPrompt("specimens", "SELECT HUMAN PLAYER SPECIMENS:", "0", "1"));
    }

    @Override
    public void init() {
        choiceMenu = new ChoiceMenu(telemetry, gamepad1, gamepad2);
        setPrompts();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //intake = new Intake(hardwareMap);
        //outtake = new Outtake(hardwareMap);
    }

    @Override
    public void init_loop(){
        choiceMenu.processPrompts();
        telemetry.update();
    }

    @Override
    public void start(){
        // Enable Auto Bulk Reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

    @Override
    public void loop() {
        switch (state) {
            case HANG_SPECIMEN:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose, alliance.equals("Blue"))
                                .splineTo(new Vector2d(10, -35), Math.PI / 2, null, new ProfileAccelConstraint(-1000, 100))
                                .build()
                );

                state = State.PICKUP_SPECIMEN;
                break;
            case PICKUP_SPECIMEN:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose, alliance.equals("Blue"))
                                .splineToSplineHeading(new Pose2d(17.07, -49.05, Math.toRadians(90)), Math.toRadians(-14.83))
                                .splineToLinearHeading(new Pose2d(41.94, -39.98, Math.toRadians(60)), Math.toRadians(67.62))
                                .splineTo(new Vector2d(46.07, -45.68), Math.toRadians(-70.35))
                                .build()
                );

                state = State.HANG_SPECIMEN;
                break;
            default:
                // Should never be reached, as state should never be null
                state = State.HANG_SPECIMEN;
                break;
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = drive.pose;
    }
}