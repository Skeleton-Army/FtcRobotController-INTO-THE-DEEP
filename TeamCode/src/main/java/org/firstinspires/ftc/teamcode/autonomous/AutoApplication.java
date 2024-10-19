package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.ChoiceMenu;
import org.firstinspires.ftc.teamcode.utils.prompts.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.prompts.ValuePrompt;

public class AutoApplication extends OpMode {
    private ChoiceMenu choiceMenu;

    @Override
    public void init() {
        choiceMenu = new ChoiceMenu(telemetry, gamepad1, gamepad2);

        choiceMenu.enqueuePrompt(new OptionPrompt("alliance", "SELECT AN ALLIANCE:", "Red", "Blue"));
        choiceMenu.enqueuePrompt(new OptionPrompt("position", "SELECT THE STARTING POSITION:", "Audience", "Rear wall"));
        choiceMenu.enqueuePrompt(new ValuePrompt("delay", "ENTER A START DELAY:", 0, 10, 0, 0.5));
    }

    @Override
    public void init_loop(){
        choiceMenu.processPrompts();
        telemetry.update();
    }

    @Override
    public void start(){
        String alliance = (String)choiceMenu.getValueOf("alliance");
        String position = (String)choiceMenu.getValueOf("position");
        double delay = (double)choiceMenu.getValueOf("delay");

        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Position", position);
        telemetry.addData("Selected Delay", delay);
    }

    @Override
    public void loop() {

    }
}