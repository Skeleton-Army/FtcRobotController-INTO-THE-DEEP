package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.skeletonarmy.marrow.AdvancedDcMotor;
import com.skeletonarmy.marrow.actions.ServoToPositionAction;
import com.skeletonarmy.marrow.actions.MotorToPositionAction;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Outtake {
    public final AdvancedDcMotor motor;
    private final CachingServo bucketServo;

    public Outtake(HardwareMap hardwareMap) {
        motor = new AdvancedDcMotor(hardwareMap.get(DcMotorEx.class, OuttakeConfig.motorName));
        motor.setCurrentLimit(OuttakeConfig.currentLimit, CurrentUnit.AMPS);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setTargetPosition(0);

        bucketServo = new CachingServo(hardwareMap.get(Servo.class, OuttakeConfig.bucketName));

        Actions.runBlocking(hold());
    }

    public void resetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // General actions
    public Action motorToPosition(int targetPos) {
        return new MotorToPositionAction(motor, targetPos);
    }

    public Action bucketToPosition(double targetPos) {
        return new ServoToPositionAction(bucketServo, targetPos);
    }

    // Specific actions
    public Action extend() {
        return motorToPosition(OuttakeConfig.extendPosition);
    }

    /**
     * Partially extend the arm by a given factor.
     * @param multiplier The factor by which to extend the arm
     */
    public Action extend(double multiplier) {
        return motorToPosition((int)(OuttakeConfig.extendPosition * multiplier));
    }

    public Action extend(boolean highBasket) {
        int pos = highBasket ? OuttakeConfig.extendPosition : OuttakeConfig.lowBasketPosition;
        return motorToPosition(pos);
    }

    public Action retract() {
        return motorToPosition(OuttakeConfig.retractPosition);
    }

    public Action dunk() {
        return bucketToPosition(OuttakeConfig.bucketDunk);
    }

    public Action filledDunk() {
        return bucketToPosition(OuttakeConfig.bucketFilledDunk);
    }

    public Action hold() {
        return bucketToPosition(OuttakeConfig.bucketHold);
    }

    public Action bucketMiddle() {
        return bucketToPosition(OuttakeConfig.bucketMiddle);
    }

    public Action bucketReady() {
        return bucketToPosition(OuttakeConfig.bucketReady);
    }
}
