package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.d;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.f;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.i;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.p;

import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.motorName;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.ticks_in_degree;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;

@TeleOp(name = "PIDF test", group = "test")
@Config
public class PIDFTest extends OpMode {
    public static int target = 0;

    private PIDController controller;

    private DcMotorEx motor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, motorName);
    }

    @Override
    public void loop() {
        target = SpecimenArmConfig.intakePosition;
        controller.setPID(p, i, d);

        int pos = motor.getCurrentPosition();
        double pid = controller.calculate(pos, target);

        double ff = Math.cos(Math.toRadians(pos / ticks_in_degree)) * f;
//        int diffFromTop = SpecimenArmConfig.topPos - pos;
//        double ff = diffFromTop * f;

        double power = pid + ff;

        motor.setPower(power);

        telemetry.addData("pos", pos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.addData("motor power: ", motor.getPower());
        telemetry.update();
    }
}
