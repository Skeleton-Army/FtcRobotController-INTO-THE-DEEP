package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
@Config
public class MotorToPosition implements Action {
    private boolean initialized = false;

    private final CachingDcMotorEx motor;
    private final int targetPos;
    private final double power;
    private final boolean holdPosition;
    private int velocityThreshold;
    private double startThreshold;
    private boolean stop;

    private final ElapsedTime timer = new ElapsedTime();

    public MotorToPosition(CachingDcMotorEx motor, int targetPos, double power, int velocityThreshold, double startThreshold, boolean holdPosition) {
        this.motor = motor;
        this.targetPos = targetPos;
        this.power = power;
        this.velocityThreshold = velocityThreshold;
        this.startThreshold = startThreshold;
        this.holdPosition = holdPosition;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            initialized = true;

            motor.setTargetPosition(targetPos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);

            timer.reset();
        }

        double currentVelocity = motor.getVelocity();
        boolean lowVelocity = Math.abs(currentVelocity) < velocityThreshold;
        boolean timeReached = timer.seconds() > startThreshold;

        telemetryPacket.put("Motor Position", motor.getCurrentPosition());
        telemetryPacket.put("Motor Velocity", currentVelocity);
        telemetryPacket.put("Timer", timer.seconds());

        boolean shouldStop = timeReached && lowVelocity;

        // Reached target position / physically stopped (End of action)
        if (shouldStop) {
            if (holdPosition) {
                motor.setTargetPosition(motor.getCurrentPosition());
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(power / 2);
            } else {
                motor.setPower(0);
            }
        }

        return !shouldStop;
    }
}
