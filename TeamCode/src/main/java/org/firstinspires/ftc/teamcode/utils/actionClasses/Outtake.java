package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.actions.MotorToPosition;
import org.firstinspires.ftc.teamcode.utils.actions.ServoToPosition;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Outtake {
    public final CachingDcMotorEx motor;
    private final Servo bucketServo;

    public Outtake(HardwareMap hardwareMap) {
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, OuttakeConfig.motorName));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setTargetPosition(0);

        bucketServo = hardwareMap.get(Servo.class, OuttakeConfig.bucketName);

        Actions.runBlocking(hold());
    }

    public void resetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // General actions
    public Action motorToPosition(int targetPos, double power, boolean holdPosition) {
        return new MotorToPosition(motor, targetPos, power, OuttakeConfig.velocityThreshold, OuttakeConfig.startThreshold, holdPosition);
    }

    public Action bucketToPosition(double targetPos) {
        return new ServoToPosition(bucketServo, targetPos);
    }

    // Specific actions
    public Action extend() {
        return motorToPosition(OuttakeConfig.extendPosition, OuttakeConfig.motorPower, true);
    }

    /**
     * Partially extend the arm by a given factor.
     * @param multiplier The factor by which to extend the arm
     */
    public Action extend(double multiplier) {
        return motorToPosition((int)(OuttakeConfig.extendPosition * multiplier), OuttakeConfig.motorPower, true);
    }

    public Action extend(boolean highBasket) {
        int pos = highBasket ? OuttakeConfig.extendPosition : OuttakeConfig.lowBasketPosition;
        return motorToPosition(pos, OuttakeConfig.motorPower, true);
    }

    public Action retract() {
        return motorToPosition(OuttakeConfig.retractPosition, OuttakeConfig.motorPower, false);
    }

    public Action dunk() {
        return bucketToPosition(OuttakeConfig.bucketDunk);
    }

    public Action hold() {
        return bucketToPosition(OuttakeConfig.bucketHold);
    }

    public Action bucketMiddle() {
        return bucketToPosition(OuttakeConfig.bucketMiddle);
    }
}
