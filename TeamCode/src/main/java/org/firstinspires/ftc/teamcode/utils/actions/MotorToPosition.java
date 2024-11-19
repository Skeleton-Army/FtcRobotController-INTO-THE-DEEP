package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorToPosition implements Action {
    private boolean initialized = false;

    private final DcMotorEx motor;
    private final int targetPos;
    private final double power;

    public MotorToPosition(DcMotorEx motor, int targetPos, double power) {
        this.motor = motor;
        this.targetPos = targetPos;
        this.power = power;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            initialized = true;

            motor.setTargetPosition(targetPos);
            motor.setPower(power);
        }

        return !motor.isBusy();
    }
}
