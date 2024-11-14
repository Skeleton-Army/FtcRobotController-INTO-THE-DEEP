package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.utils.general.Debounce;

@TeleOp(name = "Continuous Servo Test", group = "SA_FTC")
public class ContinuousServoTest extends OpMode {
    private CRServo crServo;

    private double power = 0.1;
    private CRServo.Direction direction = CRServo.Direction.FORWARD;

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, "testCRServo");
    }

    @Override
    public void loop() {
        if (Debounce.isButtonPressed(gamepad1.dpad_up)) {
            power = Math.min(1, power + 0.1);
        }
        else if (Debounce.isButtonPressed(gamepad1.dpad_down)) {
            power = Math.max(0, power - 0.1);
        }

        if (Debounce.isButtonPressed(gamepad1.a)) {
            direction = (direction == CRServo.Direction.FORWARD) ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD;
        }

        crServo.setPower(power);
        crServo.setDirection(direction);

        telemetry.addData("Speed", power);
        telemetry.update();
    }
}
