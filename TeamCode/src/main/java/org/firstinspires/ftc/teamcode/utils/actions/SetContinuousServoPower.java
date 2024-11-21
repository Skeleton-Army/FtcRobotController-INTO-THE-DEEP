package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;

public class SetContinuousServoPower implements Action  {
    private final CRServo crServo;
    private final double power;

    public SetContinuousServoPower(CRServo crServo, double power) {
        this.crServo = crServo;
        this.power = power;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        crServo.setPower(power);

        return false; // Finish immediately
    }
}
