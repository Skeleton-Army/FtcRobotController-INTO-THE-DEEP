package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ShoevTest extends OpMode {
    CRServo servo1;
    CRServo servo2;

    @Override
    public void init() {
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        servo1.setDirection(DcMotorSimple.Direction.REVERSE);
        servo2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        servo1.setPower(1);
        servo2.setPower(1);
    }
}
