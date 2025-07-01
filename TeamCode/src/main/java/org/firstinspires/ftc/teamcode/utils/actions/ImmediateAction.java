package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ImmediateAction implements Action {
    private final Action action;

    public ImmediateAction(Action action) {
        this.action = action;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        action.run(telemetryPacket);

        return false;
    }
}
