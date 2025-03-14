package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

public class ConditionAction implements Action {
    private final Action action;
    private Supplier<Boolean> condition;
    private Action conditionAction;

    /**
     * Calls an action while the condition is true.
     */
    public ConditionAction(Action action, Supplier<Boolean> condition) {
        this.action = action;
        this.condition = condition;
    }

    /**
     * Calls an action while the conditionAction is running.
     */
    public ConditionAction(Action action, Action conditionAction) {
        this.action = action;
        this.conditionAction = conditionAction;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        action.run(telemetryPacket);

        return condition != null ? condition.get() : conditionAction.run(telemetryPacket); // Re-evaluate the condition each time
    }
}
