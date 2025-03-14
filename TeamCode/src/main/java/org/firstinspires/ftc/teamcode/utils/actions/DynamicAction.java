package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

public class DynamicAction implements Action {
    private final Supplier<Action> actionSupplier;
    private Action currentAction;

    /**
     * DynamicAction is a wrapper that allows actions to be created dynamically at runtime.
     * Instead of initializing the action with a fixed value, it retrieves the action
     * when it starts running, ensuring it uses the latest data.
     */
    public DynamicAction(Supplier<Action> actionSupplier) {
        this.actionSupplier = actionSupplier;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (currentAction == null) {
            currentAction = actionSupplier.get();
        }

        return currentAction.run(telemetryPacket);
    }
}
