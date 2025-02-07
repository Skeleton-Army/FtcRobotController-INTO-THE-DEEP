package org.firstinspires.ftc.teamcode.utils.general.prompts;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;

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

        if (Utilities.isPressed(gamepad1.dpad_right || gamepad2.dpad_right) || Utilities.isPressed(gamepad1.dpad_up || gamepad2.dpad_up) || Utilities.isPressed(gamepad1.dpad_left || gamepad2.dpad_left) || Utilities.isPressed(gamepad1.dpad_down || gamepad2.dpad_down)) {
            selectedValue = !selectedValue;
        }

        if (Utilities.isPressed(gamepad1.a || gamepad2.a)) {
            return selectedValue;
        }

        return null;
    }
}
