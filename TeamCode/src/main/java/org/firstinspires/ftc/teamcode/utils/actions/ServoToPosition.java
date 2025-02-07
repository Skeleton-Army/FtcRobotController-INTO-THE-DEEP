package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoToPosition implements Action {
    private final Servo servo;
    private final double targetPos;

    public ServoToPosition(Servo servo, double targetPos) {
        this.servo = servo;
        this.targetPos = targetPos;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        servo.setPosition(targetPos);

        return false; // Finish immediately
    }
}
