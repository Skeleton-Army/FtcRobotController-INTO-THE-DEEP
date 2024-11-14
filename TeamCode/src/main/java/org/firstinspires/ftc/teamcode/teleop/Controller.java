package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOp App", group = "SA_FTC")
public class Controller extends LinearOpMode {
    public static Controller Instance;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Instance = this;

        MovementUtils movementUtils = new MovementUtils(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            movementUtils.movement(gamepad1);
            //movementUtils.fieldCentricMovement(gamepad1, telemetry);

            // Debugging
            telemetry.update();
        }
    }
}