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
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.skeletonarmy.marrow.AdvancedDcMotor;
import com.skeletonarmy.marrow.actions.ServoToPosition;
import com.skeletonarmy.marrow.actions.SleepUntilAction;

import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class SpecimenArm {
    public final AdvancedDcMotor motor;
    private final CachingServo gripServo;
    private final CachingServo grabServo;

    public SpecimenArm(HardwareMap hardwareMap) {
        motor = new AdvancedDcMotor(hardwareMap.get(DcMotorEx.class, SpecimenArmConfig.motorName));
        motor.setUseCustomPIDF(true);
        motor.setCustomPIDFCoefficients(p, i, d, f);
        motor.setCustomPIDFController(this::customPIDFController);
        motor.setTargetPosition(motor.getCurrentPosition());

        gripServo = new CachingServo(hardwareMap.get(Servo.class, SpecimenArmConfig.servoName));
        grabServo = new CachingServo(hardwareMap.get(Servo.class, SpecimenArmConfig.grabServoName));
    }

    public void resetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // General actions
    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setPID(double kp, double ki, double kd) {
        motor.setCustomPIDCoefficients(kp, ki, kd);
    }

    public Action setTarget(int target) {
        return new SequentialAction(
                new InstantAction(() -> motor.setTargetPosition(target)),
                new SleepUntilAction(() -> Math.abs(motor.getCurrentPosition() - target) < 10)
        );
    }

    public Action runManualControl(float value) {
        if (value == 0) return new NullAction();
        int mirror = motor.getCurrentPosition() < SpecimenArmConfig.topPos ? -1 : 1;
        return setTarget(motor.getTargetPosition() + (int)(value * SpecimenArmConfig.manualSpeed * mirror));
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

    public Action goToPark() {
        return setTarget(SpecimenArmConfig.parkPosition);
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

    public double customPIDFController(AdvancedDcMotor motor, int target) {
        int pos = motor.getCurrentPosition();
        PIDFController controller = motor.getPIDFController();

        double pid = controller.calculate(pos, target);

        int diffFromTop = SpecimenArmConfig.topPos - pos;
        double ff = diffFromTop * controller.getF();

        return pid + ff;
    }
}
