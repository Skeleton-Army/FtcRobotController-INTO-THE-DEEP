package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Intake {
    public final CachingDcMotorEx motor;
    private final Servo clawServo;
    private final Servo wristServo1;
    private final Servo wristServo2;
    private final Servo rotationServo;

    public Intake(HardwareMap hardwareMap) {
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, IntakeConfig.motorName));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo = hardwareMap.get(Servo.class, IntakeConfig.clawName);
        wristServo1 = hardwareMap.get(Servo.class, IntakeConfig.wrist1Name);
        wristServo2 = hardwareMap.get(Servo.class, IntakeConfig.wrist2Name);
        rotationServo = hardwareMap.get(Servo.class, IntakeConfig.rotationName);
    }

    public void resetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Manual control
    public void setPower(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    // General actions
    public Action motorToPosition(int targetPos, double power, boolean holdPosition) {
        return new MotorToPosition(motor, targetPos, power, IntakeConfig.velocityThreshold, IntakeConfig.startThreshold, holdPosition);
    }

    public Action clawToPosition(double targetPos) {
        return new ServoToPosition(clawServo, targetPos);
    }

    public Action wristToPosition(double targetPos) {
        return new ParallelAction(
                new ServoToPosition(wristServo1, targetPos),
                new ServoToPosition(wristServo2, targetPos)
        );
    }

    public Action clawToRotation(double targetPos) {
        return new ServoToPosition(rotationServo, targetPos);
    }

    // Specific actions
    public Action extend() {
        return motorToPosition(IntakeConfig.extendPosition, IntakeConfig.motorPower, true);
    }

    /**
     * Partially extend the arm by a given factor.
     * @param multiplier The factor by which to extend the arm
     */
    public Action extend(double multiplier) {
        return motorToPosition((int)(IntakeConfig.extendPosition * multiplier), IntakeConfig.motorPower, true);
    }

    public Action retract() {
        return motorToPosition(IntakeConfig.retractPosition, IntakeConfig.motorPower, false);
    }

    public Action retract(double power) {
        return motorToPosition(IntakeConfig.retractPosition, power, false);
    }

    public Action closeClaw() {
        return clawToPosition(IntakeConfig.clawClosed);
    }

    public Action openClaw() {
        return clawToPosition(IntakeConfig.clawOpen);
    }

    public Action extraOpenClaw() {
        return clawToPosition(IntakeConfig.extraOpenClaw);
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

    public Action wristReady() {
        return wristToPosition(IntakeConfig.wristReady);
    }

    public Action rotate(double input) {
        return clawToRotation(Utilities.remap(input, -1, 0, 1, IntakeConfig.rotationLeft, IntakeConfig.rotationForward, IntakeConfig.rotationRight));
    }
}
