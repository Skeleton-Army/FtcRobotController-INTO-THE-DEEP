package org.firstinspires.ftc.teamcode.utils.prompts;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        telemetry.addData("Increment", increment);
        telemetry.addLine("[" + minValue + "] " + selectedValue + " [" + maxValue + "]");

        if (isDpadRightPressed(gamepad1, gamepad2) || isDpadUpPressed(gamepad1, gamepad2)) {
            selectedValue = Math.min(maxValue, selectedValue + increment);
        } else if (isDpadLeftPressed(gamepad1, gamepad2) || isDpadDownPressed(gamepad1, gamepad2)) {
            selectedValue = Math.max(minValue, selectedValue - increment);
        }

        if (isAButtonPressed(gamepad1, gamepad2)) {
            return selectedValue;
        }

        return null;
    }
}
