package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.actions.MotorToPosition;

public class Intake {
    private final DcMotorEx intakeExtend;

    public Intake(HardwareMap hardwareMap) {
        intakeExtend = hardwareMap.get(DcMotorEx.class, "extend");
        intakeExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Action moveArm(int targetPos, double power) {
        return new MotorToPosition(intakeExtend, targetPos, power);
    }
}
