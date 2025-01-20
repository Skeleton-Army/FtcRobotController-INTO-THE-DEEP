package org.firstinspires.ftc.teamcode.utils.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.general.ChoiceMenu;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;

import java.util.HashMap;
import java.util.Map;

/*
    An OpMode that implements the choice menu and FSM (Finite State Machine).
    The base enhanced OpMode for autonomous programs.
 */
public abstract class AutoOpMode extends OpMode {
    private final Map<Enum<?>, Runnable> stateHandlers = new HashMap<>();

    private Enum<?> currentState = null;

    protected ChoiceMenu choiceMenu;
    protected MecanumDrive drive;

    // Abstract method to set the prompts
    public abstract void setPrompts();

    // Abstract method for subclasses to register their states
    protected abstract void registerStates();

    // Abstract method to set the initial state
    protected abstract Enum<?> initialState();

    @Override
    public void init() {
        choiceMenu = new ChoiceMenu(telemetry, gamepad1, gamepad2);
        setPrompts();

        registerStates();
        currentState = initialState();
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

        telemetry.update();
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
