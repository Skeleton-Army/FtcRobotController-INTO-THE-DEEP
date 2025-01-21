package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.actions.MotorToPosition;
import org.firstinspires.ftc.teamcode.utils.config.HangConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Hang {
    private final CachingDcMotorEx hangMotor;
    private final CachingDcMotorEx outtakeMotor;

    public Hang(HardwareMap hardwareMap) {
        hangMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, HangConfig.hangMotorName));
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, OuttakeConfig.motorName));
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetMotor() {
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // General actions
    public Action hangToPosition(int targetPos, double power, boolean holdPosition) {
        return new MotorToPosition(hangMotor, targetPos, power, HangConfig.velocityThreshold, HangConfig.startThreshold, holdPosition);
    }

    public Action outtakeToPosition(int targetPos, double power, boolean holdPosition) {
        return new MotorToPosition(outtakeMotor, targetPos, power, HangConfig.velocityThreshold, HangConfig.startThreshold, holdPosition);
    }

    // Specific actions
    public Action extendHang() {
        return hangToPosition(HangConfig.hangExtendPosition, HangConfig.hangPower, true);
    }

    public Action retractHang() {
        return hangToPosition(HangConfig.hangRetractPosition, HangConfig.hangPower, true);
    }

    public Action extendOuttake() {
        return outtakeToPosition(HangConfig.outtakeExtendPosition, HangConfig.outtakePower, true);
    }

    public Action retractOuttake() {
        return outtakeToPosition(HangConfig.outtakeRetractPosition, HangConfig.outtakePower, true);
    }
}
