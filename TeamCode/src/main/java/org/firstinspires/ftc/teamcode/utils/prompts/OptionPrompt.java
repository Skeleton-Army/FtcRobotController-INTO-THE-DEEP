package org.firstinspires.ftc.teamcode.utils.prompts;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.ChoiceMenuInput;

public class OptionPrompt extends Prompt {
    private final String[] options;
    private int selectedOptionIndex = 0;

    public OptionPrompt(String key, String header, String... options) {
        super(key, header);
        this.options = options;
    }

    @Override
    public Object process(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        telemetry.addLine(header);
        telemetry.addLine();

        for (int i = 0; i < options.length; i++) {
            if (i == selectedOptionIndex) {
                telemetry.addLine((i + 1) + ") " + options[i] + " <");
            } else {
                telemetry.addLine((i + 1) + ") " + options[i]);
            }
        }

        if (ChoiceMenuInput.isDpadRightPressed(gamepad1, gamepad2) || ChoiceMenuInput.isDpadUpPressed(gamepad1, gamepad2)) {
            selectedOptionIndex = (selectedOptionIndex - 1 + options.length) % options.length;
        } else if (ChoiceMenuInput.isDpadLeftPressed(gamepad1, gamepad2) || ChoiceMenuInput.isDpadDownPressed(gamepad1, gamepad2)) {
            selectedOptionIndex = (selectedOptionIndex + 1) % options.length;
        }

        if (ChoiceMenuInput.isAButtonPressed(gamepad1, gamepad2)) {
            return options[selectedOptionIndex];
        }

        return null;
    }
}
