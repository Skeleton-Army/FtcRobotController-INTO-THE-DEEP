package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.actions.MotorToPosition;
import org.firstinspires.ftc.teamcode.utils.actions.ServoToPosition;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Intake {
    private final CachingDcMotorEx intakeMotor;
    private final Servo clawServo;
    private final Servo wristServo;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, IntakeConfig.motorName));
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawServo = hardwareMap.get(Servo.class, IntakeConfig.clawName);
        wristServo = hardwareMap.get(Servo.class, IntakeConfig.wristName);
    }

    // Manual control
    public void setPower(double power) {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(power);
    }

    // General actions
    public Action motorToPosition(int targetPos, double power, boolean holdPosition) {
        return new MotorToPosition(intakeMotor, targetPos, power, holdPosition);
    }

    public Action clawToPosition(double targetPos) {
        return new ServoToPosition(clawServo, targetPos);
    }

    public Action wristToPosition(double targetPos) {
        return new ServoToPosition(wristServo, targetPos);
    }

    // Specific actions
    public Action extend() {
        return motorToPosition(IntakeConfig.extendPosition, IntakeConfig.motorPower, true);
    }

    public Action retract() {
        return motorToPosition(IntakeConfig.retractPosition, IntakeConfig.motorPower, false);
    }

    public Action closeClaw() {
        return clawToPosition(IntakeConfig.clawClosed);
    }

    public Action openClaw() {
        return clawToPosition(IntakeConfig.clawOpen);
    }

    public Action clawDeposit() {
        return clawToPosition(IntakeConfig.clawDeposit);
    }

    public Action extendWrist() {
        return wristToPosition(IntakeConfig.wristExtend);
    }

    public Action retractWrist() {
        return wristToPosition(IntakeConfig.wristRetract);
    }

    public Action wristMiddle() {
        return wristToPosition(IntakeConfig.wristMiddle);
    }
}
