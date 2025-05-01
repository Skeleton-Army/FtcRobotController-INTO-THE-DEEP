package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

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
    public static String motorName = "";
    public static int target = 0;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

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

//        int diffFromTop = SpecimenArmConfig.topPos - pos;
//        double ff = diffFromTop * f;

//        double power = pid + ff;

        double power = pid + f;

        motor.setPower(power);

        telemetry.addData("pos", pos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.addData("motor power: ", motor.getPower());
        telemetry.update();
    }
}
