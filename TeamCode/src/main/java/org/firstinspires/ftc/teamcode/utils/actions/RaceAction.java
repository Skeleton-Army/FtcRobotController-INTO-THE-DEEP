package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.Arrays;
import java.util.List;

public class RaceAction implements Action {
    private final List<Action> actions;

    /**
     * Executes multiple actions in parallel and stops as soon as any one of them completes.
     * Remaining actions will not be updated once one finishes.
     */
    public RaceAction(Action... actions) {
        this.actions = Arrays.asList(actions);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        boolean anyCompleted = false;

        // Update each action and check if any one has completed
        for (Action action : actions) {
            if (!action.run(telemetryPacket)) {
                anyCompleted = true;
            }
        }

        return !anyCompleted;
    }
}
