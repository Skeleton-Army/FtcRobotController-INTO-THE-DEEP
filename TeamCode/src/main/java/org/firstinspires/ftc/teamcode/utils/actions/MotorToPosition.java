package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class MotorToPosition implements Action {
    private final int VELOCITY_THRESHOLD = 5;

    private boolean initialized = false;

    private final CachingDcMotorEx motor;
    private final int targetPos;
    private final double power;

    public MotorToPosition(CachingDcMotorEx motor, int targetPos, double power) {
        this.motor = motor;
        this.targetPos = targetPos;
        this.power = power;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            initialized = true;

            motor.setTargetPosition(targetPos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        double currentVelocity = motor.getVelocity();
        boolean isStopped = Math.abs(currentVelocity) < VELOCITY_THRESHOLD;

        telemetryPacket.put("Motor Position", motor.getCurrentPosition());
        telemetryPacket.put("Motor Velocity", currentVelocity);

        return !isStopped;
    }
}
