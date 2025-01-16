package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.d;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.f;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.i;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.motorName;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.p;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.target;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.ticks_in_degree;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;

@TeleOp(name = "PIDF test", group = "test")
public class PIDFTest extends OpMode {
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
        controller.setPID(p, i, d);
        int pos = motor.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        double limitPower = (Math.abs((Math.cos(Math.toRadians((pos + 50) / 2)) )) * 2 * SpecimenArmConfig.power) + 0.15;
        double actualPower = clamp(power, -limitPower, limitPower);

        motor.setPower(actualPower);

        telemetry.addData("pos", pos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.addData("actual power", actualPower);
        telemetry.update();
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
