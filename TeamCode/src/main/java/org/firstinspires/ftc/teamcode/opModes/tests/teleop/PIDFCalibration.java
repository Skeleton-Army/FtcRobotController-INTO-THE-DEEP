package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/*
    A tool for calibrating the PIDF constants of a motor controller.

    RUN_TO_POSITION mode makes use of both the coefficients set for RUN_TO_POSITION (setPositionPIDFCoefficients)
    and the coefficients set for RUN_WITH_ENCODER (setVelocityPIDFCoefficients)

    Just the P term is used to control the position loop.
    The PIDF coefficients are used to obtain the desired velocity.

    https://youtu.be/E6H6Nqe6qJo?si=uIy-0d7pSVbfyAzb&t=442
 */
@Config
@TeleOp
public class PIDFCalibration extends OpMode {
    public static double p = 0, i = 0, d = 0, f = 0;
    public static double tP = 0;

    public static int target = 500;
    public static int power = 1;
    public static String motorName = "";

    DcMotorEx motor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, motorName);
    }

    @Override
    public void loop() {
        motor.setTargetPosition(target);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setVelocityPIDFCoefficients(p, i, d, f);
        motor.setPositionPIDFCoefficients(tP);
        motor.setPower(power);

        telemetry.addData("Current Position", motor.getCurrentPosition());
        telemetry.addData("Target Position", target);
        telemetry.update();
    }
}
