package org.firstinspires.ftc.teamcode.utils.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.HashMap;
import java.util.Map;

/*
    An OpMode that integrates RR actions.
    The base enhanced OpMode for teleop programs.
 */
public abstract class TeleopOpMode extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private HashMap<String, Action> runningActions = new HashMap<>();
    private final Map<String, Boolean> actionStates = new HashMap<>();

    /**
     * Run all queued actions. Call this at the end of the loop function.
     */
    protected void runAllActions() {
        TelemetryPacket packet = new TelemetryPacket();

        // Update running actions
        HashMap<String, Action> newActions = new HashMap<>();

        for (Map.Entry<String, Action> entry : runningActions.entrySet()) {
            Action action = entry.getValue();
            String name = entry.getKey();

            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.put(name, action);
            }
        }

        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }

    /**
     * Run an action without blocking the main loop.
     */
    protected void runAction(Action action) {
        runningActions.put("", action);
    }

    /**
     * Run an action without blocking the main loop.
     * The name is used for stopping the action.
     */
    protected void runAction(String name, Action action) {
        runningActions.put(name, action);
    }

    /**
     * Run two actions by toggling between them without blocking the main loop.
     * The name is used for managing the toggle state and stopping the action.
     */
    protected void runToggleAction(String name1, Action action1, String name2, Action action2) {
        // Check if the toggle state exists; if not, initialize it.
        if (!actionStates.containsKey(name1)) {
            actionStates.put(name1, false);
            actionStates.put(name2, true);
        }

        // Get the current state of the toggle.
        boolean toggle = actionStates.get(name1);
        Action action = !toggle ? action1 : action2;
        String name = !toggle ? name1 : name2;

        // Stop the previous action.
        String previousActionName = !toggle ? name2 : name1;

        stopAction(previousActionName);

        // Run the appropriate action based on the toggle state.
        runAction(name, action);

        // Toggle the state for the next call.
        actionStates.put(name1, !actionStates.get(name1));
        actionStates.put(name2, !actionStates.get(name2));
    }

    /**
     * Stop a specific action.
     */
    protected void stopAction(String name) {
        runningActions.remove(name);
    }

    /**
     * Stop all running actions.
     */
    protected void stopAllActions() {
        runningActions.clear();
    }

    /**
     * Check if an action is running.
     */
    protected boolean isActionRunning(String name) {
        return runningActions.containsKey(name);
    }

    /**
     * Get the current state of the toggle.
     */
    protected boolean isInState(String name) {
        return actionStates.get(name);
    }
}
