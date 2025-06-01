package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.skeletonarmy.marrow.AdvancedDcMotor;
import com.skeletonarmy.marrow.actions.MotorToPositionAction;

import org.firstinspires.ftc.teamcode.utils.config.HangConfig;

public class Hang {
    public final AdvancedDcMotor motor;

    public Hang(HardwareMap hardwareMap) {
        motor = new AdvancedDcMotor(hardwareMap.get(DcMotorEx.class, HangConfig.motorName));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // General actions
    public Action hangToPosition(int targetPos) {
        return new MotorToPositionAction(motor, targetPos);
    }

    // Specific actions
    public Action extendHang() {
        return hangToPosition(HangConfig.extendPosition);
    }

    public Action retractHang() {
        return hangToPosition(HangConfig.retractPosition);
    }

    public Action middleHang() {
        return hangToPosition(HangConfig.middlePosition);
    }
}
