package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;
import java.util.function.Supplier;

public class LoopAction implements Action {
    private final Supplier<Action> loopAction;
    private final Supplier<Action> intervalAction;
    private final Supplier<Action> endAction;
    private double interval;
    private final double intervalDivision;
    private final double minInterval;
    private final double timeout;
    private final Supplier<Boolean> endCondition;

    private Action currentAction;

    private final ElapsedTime totalTimer = new ElapsedTime();
    private final ElapsedTime intervalTimer = new ElapsedTime();

    private boolean initialized = false;

    /**
     * Creates a looping action that repeatedly executes a primary action while periodically running an interval action.
     * When the total time is reached, an end action is executed.
     *
     * @param loopAction       The main action that runs continuously in a loop.
     * @param intervalAction   The action that runs periodically at the specified interval.
     * @param endAction        The action that runs once when the total time is reached.
     * @param interval         The interval (in seconds) at which the interval action runs.
     * @param intervalDivision The number of which to divide the interval every interval.
     * @param minInterval      The minimum an interval can be.
     * @param timeout          The total time (in seconds) before the loop ends and the end action executes.
     */
    public LoopAction(Supplier<Action> loopAction, Supplier<Action> intervalAction, Supplier<Action> endAction, double interval, double intervalDivision, double minInterval, double timeout) {
        this.loopAction = loopAction;
        this.intervalAction = intervalAction;
        this.endAction = endAction;
        this.interval = interval;
        this.intervalDivision = intervalDivision;
        this.minInterval = minInterval;
        this.timeout = timeout;
        this.endCondition = null;
    }

    /**
     * Creates a looping action that repeatedly executes a primary action while periodically running an interval action.
     * When the total time is reached, an end action is executed.
     *
     * @param loopAction       The main action that runs continuously in a loop.
     * @param intervalAction   The action that runs periodically at the specified interval.
     * @param endAction        The action that runs once when the total time is reached.
     * @param interval         The interval (in seconds) at which the interval action runs.
     * @param intervalDivision The number of which to divide the interval every interval.
     * @param minInterval      The minimum an interval can be.
     * @param timeout          The total time (in seconds) before the loop ends and the end action executes.
     * @param endCondition     If this condition is true, the action ends.
     */
    public LoopAction(Supplier<Action> loopAction, Supplier<Action> intervalAction, Supplier<Action> endAction, double interval, double intervalDivision, double minInterval, double timeout, Supplier<Boolean> endCondition) {
        this.loopAction = loopAction;
        this.intervalAction = intervalAction;
        this.endAction = endAction;
        this.interval = interval;
        this.intervalDivision = intervalDivision;
        this.minInterval = minInterval;
        this.timeout = timeout;
        this.endCondition = endCondition;
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
            interval /= intervalDivision;
            if (interval < minInterval) interval = minInterval;

            intervalAction.get().run(telemetryPacket);
            currentAction = loopAction.get();
        }

        if (currentAction != null && !currentAction.run(telemetryPacket))
            currentAction = null;

        boolean ended = totalTimer.seconds() > timeout || Objects.requireNonNull(endCondition).get();

        if (ended)
            endAction.get().run(telemetryPacket);

        return !ended;
    }
}
