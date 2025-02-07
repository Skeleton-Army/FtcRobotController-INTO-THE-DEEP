package org.firstinspires.ftc.teamcode.utils.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.general.ChoiceMenu;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
    An OpMode that implements the choice menu and FSM (Finite State Machine).
    The base enhanced OpMode for autonomous programs.
 */
public abstract class AutoOpMode extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final Map<Enum<?>, Runnable> stateHandlers = new HashMap<>();
    private List<Action> runningActions = new ArrayList<>();
    private List<Runnable> runningFunctions = new ArrayList<>();

    private Enum<?> currentState = null;

    protected ChoiceMenu choiceMenu;
    protected MecanumDrive drive;

    // Abstract method to set the prompts
    public abstract void setPrompts();

    // Abstract method for subclasses to register their states
    protected abstract void registerStates();

    // Abstract method to set the initial state
    public abstract void setInitialState();

    @Override
    public void init() {
        choiceMenu = new ChoiceMenu(telemetry, gamepad1, gamepad2);
        setPrompts();

        registerStates();
    }

    @Override
    public void init_loop(){
        choiceMenu.processPrompts();
        telemetry.update();
    }

    @Override
    public void loop() {
        if (currentState != null) {
            Runnable handler = stateHandlers.get(currentState);

            if (handler != null) {
                telemetry.addData("State", currentState.toString());
                handler.run();
            } else {
                telemetry.addData("Error", "No handler for state: " + currentState.toString());
            }
        }

        runAsyncActions();
        runAsyncFunctions();

        telemetry.update();
    }

    /**
     * Run all queued actions.
     */
    private void runAsyncActions() {
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
     * Run all queued functions.
     */
    private void runAsyncFunctions() {
        for (Runnable func : runningFunctions) {
            func.run();
        }
    }

    /**
     * Run an action in a blocking loop.
     */
    protected void runBlocking(Action action) {
        TelemetryPacket packet = new TelemetryPacket();

        while (action.run(packet)) {
            runAsyncActions();
            runAsyncFunctions();
        }
    }

    /**
     * Run an action in a non-blocking loop.
     */
    protected void runAsync(Action action) {
        runningActions.add(action);
    }

    /**
     * Run a function in a non-blocking loop. <br>
     * <b>Example:</b> runAsync(() -> { function(); });
     * @param func The function to run asynchronously
     */
    protected void runAsync(Runnable func) {
        runningFunctions.add(func);
    }

    private void setState(Enum<?> newState) {
        currentState = newState;
    }

    /**
     * Adds a state handler to the FSM.
     * @param state The state to add the handler for
     * @param handler The function to add
     */
    protected void addState(Enum<?> state, Runnable handler) {
        stateHandlers.put(state, handler);
    }

    /**
     * Adds a transition to the FSM.
     * @param newState The state to transition to
     */
    protected void addTransition(Enum<?> newState) {
        setState(newState);
    }

    /**
     * Adds a conditional transition to the FSM.
     * @param newState The state to transition to
     * @param condition The condition to check
     */
    protected void addConditionalTransition(boolean condition, Enum<?> newState) {
        if (condition) {
            setState(newState);
        }
    }

    /**
     * Adds a conditional transition to the FSM.
     * @param trueState The state to transition to if the condition is true
     * @param falseState The state to transition to if the condition is false
     * @param condition The condition to check
     */
    protected void addConditionalTransition(boolean condition, Enum<?> trueState, Enum<?> falseState) {
        setState(condition ? trueState : falseState);
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = drive.pose;
    }
}
