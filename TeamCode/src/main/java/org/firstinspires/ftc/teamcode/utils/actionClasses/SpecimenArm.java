package org.firstinspires.ftc.teamcode.utils.actionClasses;

import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.d;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.f;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.i;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.p;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.ticks_in_degree;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.actions.ServoToPosition;
import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class SpecimenArm {
    private final CachingDcMotorEx motor;
    public final Servo gripServo;
    public final Servo grabServo;
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
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void setTarget(int target) {
        this.target = target;
    }

    public Action gripToPosition(double targetPos, Servo servo) {
        return new ServoToPosition(servo, targetPos);
    }

    // Specific actions
    public Action gripToIntake() {
        return gripToPosition(SpecimenArmConfig.gripIntake, gripServo);
    }

    public Action gripToOuttake() {
        return gripToPosition(SpecimenArmConfig.gripOuttake, gripServo);
    }

    public Action grabOpen() {return gripToPosition(SpecimenArmConfig.grabClose, grabServo);}
    public Action grabClose() {return gripToPosition(SpecimenArmConfig.grabOpen, grabServo);}

    public double calculateArmPower() {
        int pos = motor.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        double limitPower = (Math.abs((Math.cos(Math.toRadians((pos + 50) / 2.0)) )) * 2 * SpecimenArmConfig.power) + 0.7;
        double actualPower = clamp(power, -limitPower, limitPower);

        return actualPower;
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
