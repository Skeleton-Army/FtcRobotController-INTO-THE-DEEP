package org.firstinspires.ftc.teamcode.utils.actionClasses;

import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.d;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.f;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.i;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.p;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.actions.ConditionAction;
import org.firstinspires.ftc.teamcode.utils.actions.ServoToPosition;
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class SpecimenArm {
    public final CachingDcMotorEx motor;
    private final Servo gripServo;
    private final Servo grabServo;
    private final PIDController controller;

    private int target;

    public SpecimenArm(HardwareMap hardwareMap) {
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, SpecimenArmConfig.motorName));
        gripServo = hardwareMap.get(Servo.class, SpecimenArmConfig.servoName);
        grabServo = hardwareMap.get(Servo.class, SpecimenArmConfig.grabServoName);

        controller = new PIDController(p, i, d);

        target = motor.getCurrentPosition();
    }

    public void resetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // General actions
    /**
     * Updates the motor power. Call this every loop.
     */
    public void update() {
        motor.setPower(calculateArmPower());
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setPID(double kp, double ki, double kd) {
        controller.setPID(kp, ki, kd);
    }

    public Action setTarget(int target) {
        return new SequentialAction(
                new InstantAction(() -> this.target = target),
                new SleepUntilAction(() -> Math.abs(motor.getCurrentPosition() - target) < 10)
        );
    }

    public Action runManualControl(float value) {
        if (value == 0) return new NullAction();
        int mirror = motor.getCurrentPosition() < SpecimenArmConfig.topPos ? -1 : 1;
        return setTarget(target + (int)(value * SpecimenArmConfig.manualSpeed * mirror));
    }

    public Action gripToPosition(double targetPos) {
        return new ServoToPosition(gripServo, targetPos);
    }

    public Action grabToPosition(double targetPos) {
        return new ServoToPosition(grabServo, targetPos);
    }

    // Specific actions
    public Action goToIntake() {
        return setTarget(SpecimenArmConfig.intakePosition);
    }

    public Action goToOuttake() {
        return setTarget(SpecimenArmConfig.outtakePosition);
    }

    public Action goToHanged() {
        return setTarget(SpecimenArmConfig.hangedPosition);
    }

    public Action gripToIntake() {
        return gripToPosition(SpecimenArmConfig.gripIntake);
    }

    public Action gripToOuttake() {
        return gripToPosition(SpecimenArmConfig.gripOuttake);
    }

    public Action grabOpen() {
        return grabToPosition(SpecimenArmConfig.grabOpen);
    }

    public Action grabClose() {
        return grabToPosition(SpecimenArmConfig.grabClose);
    }

    public double calculateArmPower() {
        int pos = motor.getCurrentPosition();
        double pid = controller.calculate(pos, target);

        int diffFromTop = SpecimenArmConfig.topPos - pos;
        double ff = diffFromTop * f;

        return pid + ff;
    }
}
