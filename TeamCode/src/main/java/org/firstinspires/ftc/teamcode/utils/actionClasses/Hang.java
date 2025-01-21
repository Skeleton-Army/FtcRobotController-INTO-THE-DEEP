package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.actions.MotorToPosition;
import org.firstinspires.ftc.teamcode.utils.config.HangConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Hang {
    private final CachingDcMotorEx motor;

    public Hang(HardwareMap hardwareMap) {
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, HangConfig.motorName));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // General actions
    public Action motorToPosition(int targetPos, double power, boolean holdPosition) {
        return new MotorToPosition(motor, targetPos, power, HangConfig.velocityThreshold, HangConfig.startThreshold, holdPosition);
    }

    // Specific actions
    public Action extend() {
        return motorToPosition(HangConfig.extendPosition, HangConfig.motorPower, true);
    }

    public Action retract() {
        return motorToPosition(HangConfig.retractPosition, HangConfig.motorPower, true);
    }
}
