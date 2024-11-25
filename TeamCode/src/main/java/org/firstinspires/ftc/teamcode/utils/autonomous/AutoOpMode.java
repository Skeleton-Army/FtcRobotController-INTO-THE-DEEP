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

    protected Enum<?> currentState = null;

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
        Runnable handler = stateHandlers.get(currentState);

        if (handler != null) {
            handler.run();
        } else {
            telemetry.addData("Error", "No handler for state: " + currentState);
        }

        telemetry.update();
    }

    // Method for subclasses to register state handlers
    protected void addState(Enum<?> state, Runnable handler) {
        stateHandlers.put(state, handler);
    }

    // Allow subclasses to transition states
    protected void setState(Enum<?> newState) {
        currentState = newState;
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = drive.pose;
    }
}
