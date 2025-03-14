package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import static org.firstinspires.ftc.teamcode.utils.config.IntakeConfig.wristExtend;
import static org.firstinspires.ftc.teamcode.utils.config.IntakeConfig.wristMiddle;
import static org.firstinspires.ftc.teamcode.utils.config.IntakeConfig.wristRetract;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

@TeleOp(name = "WristTest", group = "test")
public class WristTest extends OpMode {

    private Servo wristServo1;
    private Servo wristServo2;


    @Override
    public void init() {
        wristServo1 = hardwareMap.get(Servo.class, "wrist1");
        wristServo2 = hardwareMap.get(Servo.class, "wrist2");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            wristServo1.setPosition(wristExtend);
            wristServo2.setPosition(wristExtend);
        }
        if (gamepad1.b) {
            wristServo1.setPosition(wristMiddle);
            wristServo2.setPosition(wristMiddle);
        }
        if (gamepad1.x) {
            wristServo1.setPosition(wristRetract);
            wristServo2.setPosition(wristRetract);
        }
    }
}
