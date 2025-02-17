package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.actions.MotorToPosition;
import org.firstinspires.ftc.teamcode.utils.config.HangConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Hang {
    public final CachingDcMotorEx motor;

    public Hang(HardwareMap hardwareMap) {
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, HangConfig.motorName));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // General actions
    public Action hangToPosition(int targetPos, double power, boolean holdPosition) {
        return new MotorToPosition(motor, targetPos, power, HangConfig.velocityThreshold, HangConfig.startThreshold, holdPosition);
    }

    // Specific actions
    public Action extendHang() {
        return hangToPosition(HangConfig.extendPosition, HangConfig.hangPower, true);
    }

    public Action retractHang() {
        return hangToPosition(HangConfig.retractPosition, HangConfig.hangPower, true);
    }
}
