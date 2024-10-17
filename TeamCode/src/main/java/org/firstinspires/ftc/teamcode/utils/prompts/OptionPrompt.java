package org.firstinspires.ftc.teamcode.utils.prompts;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

        for (int i = 0; i < options.length; i++) {
            if (i == selectedOptionIndex) {
                telemetry.addLine((i + 1) + ") " + options[i] + " <");
            } else {
                telemetry.addLine((i + 1) + ") " + options[i]);
            }
        }

        if (isDpadRightPressed(gamepad1, gamepad2) || isDpadUpPressed(gamepad1, gamepad2)) {
            selectedOptionIndex = (selectedOptionIndex - 1 + options.length) % options.length;
        } else if (isDpadLeftPressed(gamepad1, gamepad2) || isDpadDownPressed(gamepad1, gamepad2)) {
            selectedOptionIndex = (selectedOptionIndex + 1) % options.length;
        }

        if (isAButtonPressed(gamepad1, gamepad2)) {
            return options[selectedOptionIndex];
        }

        return null;
    }
}
