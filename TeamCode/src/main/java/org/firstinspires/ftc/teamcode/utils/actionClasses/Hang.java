package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.skeletonarmy.marrow.AdvancedDcMotor;
import com.skeletonarmy.marrow.actions.MotorToPosition;

import org.firstinspires.ftc.teamcode.utils.config.HangConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Hang {
    public final CachingDcMotorEx motor;

    public Hang(HardwareMap hardwareMap) {
        motor = new AdvancedDcMotor(hardwareMap.get(DcMotorEx.class, HangConfig.motorName));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // General actions
    public Action hangToPosition(int targetPos, double power) {
        return new MotorToPosition(motor, targetPos, power, HangConfig.velocityThreshold, false);
    }

    // Specific actions
    public Action extendHang() {
        return hangToPosition(HangConfig.extendPosition, HangConfig.hangPower);
    }

    public Action retractHang() {
        return hangToPosition(HangConfig.retractPosition, HangConfig.hangPower);
    }

    public Action middleHang() {
        return hangToPosition(HangConfig.middlePosition, HangConfig.hangPower);
    }
}
