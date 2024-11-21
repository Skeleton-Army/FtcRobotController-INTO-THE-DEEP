package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.general.ChoiceMenu;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.prompts.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.general.prompts.ValuePrompt;

@Autonomous(name = "Autonomous App", group = "SA_FTC", preselectTeleOp = "Teleop App")
public class AutoApplication extends OpMode {
    public enum State {
        HANG_SPECIMEN,
        PICKUP_SPECIMEN

    };

    State state = State.HANG_SPECIMEN;

    MecanumDrive drive;

    private ChoiceMenu choiceMenu;

    @Override
    public void init() {
        choiceMenu = new ChoiceMenu(telemetry, gamepad1, gamepad2);

        choiceMenu.enqueuePrompt(new OptionPrompt("alliance", "SELECT AN ALLIANCE:", "Red", "Blue"));
        choiceMenu.enqueuePrompt(new OptionPrompt("position", "SELECT THE STARTING POSITION:", "Audience", "Rear Wall"));
        choiceMenu.enqueuePrompt(new OptionPrompt("strategy", "SELECT A STRATEGY:", "Specimens", "Yellow Basket"));
        choiceMenu.enqueuePrompt(new ValuePrompt("delay", "ENTER A START DELAY:", 0, 10, 0, 0.5));
    }

    @Override
    public void init_loop(){
        choiceMenu.processPrompts();
        telemetry.update();
    }

    @Override
    public void start(){
        String alliance = choiceMenu.getValueOf("alliance").toString();
        String position = choiceMenu.getValueOf("position").toString();
        String strategy = choiceMenu.getValueOf("strategy").toString();
        double delay = Double.parseDouble(choiceMenu.getValueOf("delay").toString());

        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Position", position);
        telemetry.addData("Selected Strategy", strategy);
        telemetry.addData("Selected Delay", delay);

        drive = new MecanumDrive(hardwareMap, new Pose2d(10, -61.5, Math.toRadians(90.00)));
    }

    @Override
    public void loop() {
        switch (state) {
            case HANG_SPECIMEN:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(10,-31), Math.toRadians(90.00))
                                .build()
                );
                state = State.PICKUP_SPECIMEN;

            case PICKUP_SPECIMEN:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(10, -32), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(27.48, -35.49), Math.toRadians(30.78))
                                .splineToLinearHeading(new Pose2d(47.41, -15.74, Math.toRadians(-90)), Math.PI / 2)
                                .splineToConstantHeading(new Vector2d(47.41, -50), Math.toRadians(270))

                                .build()
                );

                state = State.HANG_SPECIMEN;
                break;



            default:
                // should never be reached, as state should never be null
                state = State.HANG_SPECIMEN;
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = drive.pose;
    }
}