package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

public class LoopAction implements Action {
    private final Supplier<Action> action1;
    private final Supplier<Action> action2;
    private final Supplier<Action> action3;
    private final double interval;
    private final double time;

    private Action currentAction;

    private final ElapsedTime totalTimer = new ElapsedTime();
    private final ElapsedTime intervalTimer = new ElapsedTime();

    private boolean initialized = false;

    public LoopAction(Supplier<Action> action1, Supplier<Action> action2, Supplier<Action> action3, double interval, double time) {
        this.action1 = action1;
        this.action2 = action2;
        this.action3 = action3;
        this.interval = interval;
        this.time = time;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            initialized = true;

            totalTimer.reset();
            intervalTimer.reset();

            currentAction = action1.get();
        }

        if (intervalTimer.seconds() >= interval) {
            intervalTimer.reset();

            action2.get().run(telemetryPacket);
            currentAction = action1.get();
        }

        if (currentAction != null && !currentAction.run(telemetryPacket))
            currentAction = null;

        boolean ended = totalTimer.seconds() > time;

        if (ended)
            action3.get().run(telemetryPacket);

        return !ended;
    }
}
