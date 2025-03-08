package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

public class LoopAction implements Action {
    private final Supplier<Action> loopAction;
    private final Supplier<Action> intervalAction;
    private final Supplier<Action> endAction;
    private final double interval;
    private final double time;

    private Action currentAction;

    private final ElapsedTime totalTimer = new ElapsedTime();
    private final ElapsedTime intervalTimer = new ElapsedTime();

    private boolean initialized = false;

    /**
     * Creates a looping action that repeatedly executes a primary action while periodically running an interval action.
     * When the total time is reached, an end action is executed.
     *
     * @param loopAction     The main action that runs continuously in a loop.
     * @param intervalAction The action that runs periodically at the specified interval.
     * @param endAction      The action that runs once when the total time is reached.
     * @param interval       The interval (in seconds) at which the interval action runs.
     * @param time           The total time (in seconds) before the loop ends and the end action executes.
     */
    public LoopAction(Supplier<Action> loopAction, Supplier<Action> intervalAction, Supplier<Action> endAction, double interval, double time) {
        this.loopAction = loopAction;
        this.intervalAction = intervalAction;
        this.endAction = endAction;
        this.interval = interval;
        this.time = time;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            initialized = true;

            totalTimer.reset();
            intervalTimer.reset();

            currentAction = loopAction.get();
        }

        if (intervalTimer.seconds() >= interval) {
            intervalTimer.reset();

            intervalAction.get().run(telemetryPacket);
            currentAction = loopAction.get();
        }

        if (currentAction != null && !currentAction.run(telemetryPacket))
            currentAction = null;

        boolean ended = totalTimer.seconds() > time;

        if (ended)
            endAction.get().run(telemetryPacket);

        return !ended;
    }
}
