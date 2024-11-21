package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.actions.MotorToPosition;
import org.firstinspires.ftc.teamcode.utils.actions.ServoToPosition;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;

public class Outtake {
    private final DcMotorEx outtakeMotor;
    private final Servo bucketServo;

    public Outtake(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        outtakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketServo = hardwareMap.get(Servo.class, "bucket");
    }

    // General actions
    public Action motorToPosition(int targetPos, double power) {
        return new MotorToPosition(outtakeMotor, targetPos, power);
    }

    public Action bucketToPosition(double targetPos) {
        return new ServoToPosition(bucketServo, targetPos);
    }

    // Specific actions
    public Action extendIntake() {
        return motorToPosition(OuttakeConfig.extendPosition, OuttakeConfig.motorPower);
    }

    public Action retractIntake() {
        return motorToPosition(OuttakeConfig.retractPosition, OuttakeConfig.motorPower);
    }

    public Action dunk() {
        return bucketToPosition(OuttakeConfig.bucketRelease);
    }

    public Action hold() {
        return bucketToPosition(OuttakeConfig.bucketHold);
    }
}
