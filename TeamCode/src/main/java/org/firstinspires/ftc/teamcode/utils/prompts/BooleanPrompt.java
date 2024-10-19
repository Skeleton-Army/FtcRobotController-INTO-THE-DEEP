package org.firstinspires.ftc.teamcode.utils.prompts;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.ChoiceMenuInput;

public class BooleanPrompt extends Prompt {
    private boolean selectedValue;

    public BooleanPrompt(String key, String header, boolean defaultValue) {
        super(key, header);
        this.selectedValue = defaultValue;
    }

    @Override
    public Object process(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        telemetry.addLine(header);
        telemetry.addLine("Current Value: " + (selectedValue ? "Yes" : "No"));

        if (ChoiceMenuInput.isDpadRightPressed(gamepad1, gamepad2) || ChoiceMenuInput.isDpadUpPressed(gamepad1, gamepad2) || ChoiceMenuInput.isDpadLeftPressed(gamepad1, gamepad2) || ChoiceMenuInput.isDpadDownPressed(gamepad1, gamepad2)) {
            selectedValue = !selectedValue;
        }

        if (ChoiceMenuInput.isAButtonPressed(gamepad1, gamepad2)) {
            return selectedValue;
        }

        return null;
    }
}
