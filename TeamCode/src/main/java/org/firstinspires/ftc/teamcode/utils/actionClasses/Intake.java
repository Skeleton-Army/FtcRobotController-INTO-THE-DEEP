package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.actions.MotorToPosition;
import org.firstinspires.ftc.teamcode.utils.actions.ServoToPosition;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

public class Intake {
    private final DcMotorEx intakeMotor;
    private final Servo clawServo;
    private final Servo wristFrontServo;
    private final Servo wristBackServo;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConfig.motorName);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo = hardwareMap.get(Servo.class, IntakeConfig.clawName);
        wristFrontServo = hardwareMap.get(Servo.class, IntakeConfig.wristFrontName);
        wristBackServo = hardwareMap.get(Servo.class, IntakeConfig.wristBackName);
    }

    // General actions
    public Action motorToPosition(int targetPos, double power) {
        return new MotorToPosition(intakeMotor, targetPos, power);
    }

    public Action clawToPosition(double targetPos) {
        return new ServoToPosition(clawServo, targetPos);
    }

    public Action wristFrontToPosition(double targetPos) {
        return new ServoToPosition(wristFrontServo, targetPos);
    }

    public Action wristBackToPosition(double targetPos) {
        return new ServoToPosition(wristBackServo, targetPos);
    }

    // Specific actions
    public Action extend() {
        return motorToPosition(IntakeConfig.extendPosition, IntakeConfig.motorPower);
    }

    public Action retract() {
        return motorToPosition(IntakeConfig.retractPosition, IntakeConfig.motorPower);
    }

    public Action closeClaw() {
        return clawToPosition(IntakeConfig.clawClosed);
    }

    public Action openClaw() {
        return clawToPosition(IntakeConfig.clawOpen);
    }

    public Action extendFrontWrist() {
        return wristFrontToPosition(IntakeConfig.wristClosed);
    }

    public Action extendBackWrist() {
        return wristBackToPosition(IntakeConfig.wristClosed);
    }

    public Action retractFrontWrist() {
        return wristFrontToPosition(IntakeConfig.wristOpen);
    }

    public Action retractBackWrist() {
        return wristBackToPosition(IntakeConfig.wristOpen);
    }
}
