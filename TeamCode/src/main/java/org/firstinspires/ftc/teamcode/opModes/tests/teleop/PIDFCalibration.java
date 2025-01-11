package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.target;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.actions.SpecimenArmPIDF;
import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;

/*
    A tool for calibrating the PIDF constants of a motor controller.

    RUN_TO_POSITION mode makes use of both the coefficients set for RUN_TO_POSITION (setPositionPIDFCoefficients)
    and the coefficients set for RUN_WITH_ENCODER (setVelocityPIDFCoefficients)

    Just the P term is used to control the position loop.
    The PIDF coefficients are used to obtain the desired velocity.

    https://youtu.be/E6H6Nqe6qJo?si=uIy-0d7pSVbfyAzb&t=442
 */
@TeleOp(name = "specimenArm PIDF calibration", group = "test")
public class PIDFCalibration extends OpMode {

    public static String motorName = SpecimenArmConfig.motorName;

    DcMotorEx motor;

    Action specimenArmAction;
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, motorName);
        specimenArmAction = new SpecimenArmPIDF(motor);

        Actions.runBlocking(specimenArmAction);
    }

    @Override
    public void loop() {
        //Actions.runBlocking(specimenArmAction);

        telemetry.addData("Current Position", motor.getCurrentPosition());
        telemetry.addData("Target Position", target);
        telemetry.update();
    }
}
