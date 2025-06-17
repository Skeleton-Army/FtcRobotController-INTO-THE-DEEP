package org.firstinspires.ftc.teamcode.utils.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.skeletonarmy.marrow.MarrowUtils;

import java.util.HashMap;
import java.util.Map;

/*
    An OpMode that integrates RR actions.
    The base enhanced OpMode for teleop programs.
 */
public abstract class TeleopOpMode extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private HashMap<String, Action> runningActions = new HashMap<>();
    private final Map<String, Integer> sequenceStates = new HashMap<>();

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
        runAction(MarrowUtils.generateCallSiteID(), action);
    }

    /**
     * Run an action without blocking the main loop.
     * The name is used for stopping the action.
     */
    protected void runAction(String name, Action action) {
        runningActions.put(name, action);
    }

    /**
     * Runs multiple actions sequentially, toggling between them in order.
     * The name is used for checking the current state with "isInState".
     */
    protected void runSequentialActions(String name, Action... actions) {
        // Initialize sequence state if not present
        sequenceStates.putIfAbsent(name, 0);

        int index = sequenceStates.get(name);

        // Stop the previous action
        int prevIndex = (index == 0) ? actions.length - 1 : index - 1;
        stopAction(name + "_" + prevIndex);

        // Run the next action
        String currentActionName = name + "_" + index;
        runAction(currentActionName, actions[index]);

        // Update the state to cycle to the next action
        sequenceStates.put(name, (index + 1) % actions.length);
    }

    /**
     * Runs multiple actions sequentially, toggling between them in order.
     */
    protected void runSequentialActions(Action... actions) {
       runSequentialActions(MarrowUtils.generateCallSiteID(), actions);
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
     * Check if an action is running.
     */
    protected boolean isActionRunning(String sequenceName, int index) {
        return runningActions.containsKey(sequenceName + "_" + index);
    }

    /**
     * Get the current state of the toggle.
     */
    protected boolean isInState(String sequenceName, int index) {
        return getCurrentState(sequenceName) == index;
    }

    protected int getCurrentState(String sequenceName) {
        if (!sequenceStates.containsKey(sequenceName)) return -1;
        return sequenceStates.get(sequenceName);
    }
}
