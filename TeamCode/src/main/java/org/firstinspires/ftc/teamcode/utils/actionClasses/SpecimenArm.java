package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.actions.MotorToPosition;
import org.firstinspires.ftc.teamcode.utils.actions.ServoToPosition;
import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;

public class SpecimenArm {
    private final DcMotorEx motor;
    private final Servo servo;

    public SpecimenArm(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, SpecimenArmConfig.motorName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo = hardwareMap.get(Servo.class, SpecimenArmConfig.servoName);
    }

    // General actions
    public Action motorToPosition(int targetPos, double power, boolean holdPosition) {
        return new MotorToPosition(motor, targetPos, power, holdPosition);
    }

    public Action gripToPosition(double targetPos) {
        return new ServoToPosition(servo, targetPos);
    }

    // Specific actions
    public Action armToIntake() {
        return motorToPosition(SpecimenArmConfig.intakePosition, SpecimenArmConfig.motorPower, true);
    }

    public Action armToOuttake() {
        return motorToPosition(SpecimenArmConfig.outtakePosition, SpecimenArmConfig.motorPower, true);
    }

    public Action gripToIntake() {
        return gripToPosition(SpecimenArmConfig.gripIntake);
    }

    public Action gripToOuttake() {
        return gripToPosition(SpecimenArmConfig.gripOuttake);
    }
}