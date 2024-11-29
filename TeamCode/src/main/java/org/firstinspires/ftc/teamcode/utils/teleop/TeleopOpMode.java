package org.firstinspires.ftc.teamcode.utils.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

/*
    An OpMode that integrates RR actions.
    The base enhanced OpMode for teleop programs.
 */
public abstract class TeleopOpMode extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    /**
     * Run all queued actions. Call this at the end of the loop function.
     */
    protected void runAllActions() {
        TelemetryPacket packet = new TelemetryPacket();

        // Update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }

        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }

    /**
     * Run an action without blocking the main loop.
     */
    protected void runAction(Action action) {
        runningActions.add(action);
    }
}
