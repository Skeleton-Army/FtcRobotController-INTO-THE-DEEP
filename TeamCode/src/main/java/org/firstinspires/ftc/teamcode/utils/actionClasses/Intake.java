package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.skeletonarmy.marrow.AdvancedDcMotor;
import com.skeletonarmy.marrow.actions.MotorToPosition;
import com.skeletonarmy.marrow.actions.ServoToPosition;

import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Intake {
    public final AdvancedDcMotor motor;
    private final CachingServo clawServo;
    private final CachingServo wristServo1;
    private final CachingServo wristServo2;
    private final CachingServo rotationServo;

    public Intake(HardwareMap hardwareMap) {
        motor = new AdvancedDcMotor(hardwareMap.get(DcMotorEx.class, IntakeConfig.motorName));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo = new CachingServo(hardwareMap.get(Servo.class, IntakeConfig.clawName));
        wristServo1 = new CachingServo(hardwareMap.get(Servo.class, IntakeConfig.wrist1Name));
        wristServo2 = new CachingServo(hardwareMap.get(Servo.class, IntakeConfig.wrist2Name));
        rotationServo = new CachingServo(hardwareMap.get(Servo.class, IntakeConfig.rotationName));
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
        return new MotorToPosition(motor, targetPos, power, IntakeConfig.velocityThreshold, holdPosition);
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
        return clawToRotation(remap(input, -1, 0, 1, IntakeConfig.rotationLeft, IntakeConfig.rotationForward, IntakeConfig.rotationRight));
    }

    /**
     * Remaps a value from one range to another using linear interpolation.
     *
     * @param value  The input value to remap.
     * @param inMin  The lower bound of the input range.
     * @param inMax  The upper bound of the input range.
     * @param outMin The lower bound of the output range.
     * @param outMax The upper bound of the output range.
     * @return The remapped value in the output range.
     */
    private static double remap(double value, double inMin, double inMid, double inMax, double outMin, double outMid, double outMax) {
        if (value <= inMid) {
            return outMin + (value - inMin) * (outMid - outMin) / (inMid - inMin);
        } else {
            return outMid + (value - inMid) * (outMax - outMid) / (inMax - inMid);
        }
    }
}
