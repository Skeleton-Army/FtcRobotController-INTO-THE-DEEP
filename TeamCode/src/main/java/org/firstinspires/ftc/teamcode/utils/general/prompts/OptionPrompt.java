package org.firstinspires.ftc.teamcode.utils.general.prompts;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;

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

        if (Utilities.isPressed(gamepad1.dpad_up || gamepad2.dpad_up)) {
            selectedOptionIndex = (selectedOptionIndex - 1 + options.length) % options.length;
        } else if (Utilities.isPressed(gamepad1.dpad_down || gamepad2.dpad_down)) {
            selectedOptionIndex = (selectedOptionIndex + 1) % options.length;
        }

        if (Utilities.isPressed(gamepad1.a || gamepad2.a)) {
            return options[selectedOptionIndex];
        }

        return null;
    }
}
