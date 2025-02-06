package org.firstinspires.ftc.teamcode.utils.general.prompts;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;

public class ValuePrompt extends Prompt {
    private final double minValue;
    private final double maxValue;
    private final double increment;
    private double selectedValue;

    public ValuePrompt(String key, String header, double minValue, double maxValue, double defaultValue, double increment) {
        super(key, header);
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.increment = increment;
        this.selectedValue = defaultValue;
    }

    @Override
    public Object process(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        telemetry.addLine(header);
        telemetry.addLine();

        telemetry.addData("Increment", increment);
        telemetry.addLine("[" + minValue + "] " + selectedValue + " [" + maxValue + "]");

        if (Utilities.isPressed(gamepad1.dpad_right || gamepad2.dpad_right) || Utilities.isPressed(gamepad1.dpad_up || gamepad2.dpad_up)) {
            selectedValue = Math.min(maxValue, selectedValue + increment);
        } else if (Utilities.isPressed(gamepad1.dpad_left || gamepad2.dpad_left) || Utilities.isPressed(gamepad1.dpad_down || gamepad2.dpad_down)) {
            selectedValue = Math.max(minValue, selectedValue - increment);
        }

        if (Utilities.isPressed(gamepad1.a || gamepad2.a)) {
            return selectedValue;
        }

        return null;
    }
}
