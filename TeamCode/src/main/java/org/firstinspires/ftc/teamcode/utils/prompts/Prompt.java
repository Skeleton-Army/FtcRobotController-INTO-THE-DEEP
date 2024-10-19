package org.firstinspires.ftc.teamcode.utils.prompts;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Prompt {
    protected final String key;
    protected final String header;

    public Prompt(String key, String header) {
        this.key = key;
        this.header = header;
    }

    public String getKey() {
        return key;
    }

    public abstract Object process(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry);

    // Helper functions for checking inputs
    protected boolean isDpadLeftPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return gamepad1.dpad_left || gamepad2.dpad_left;
    }

    protected boolean isDpadRightPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return gamepad1.dpad_right || gamepad2.dpad_right;
    }

    protected boolean isDpadUpPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return gamepad1.dpad_up || gamepad2.dpad_up;
    }

    protected boolean isDpadDownPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return gamepad1.dpad_down || gamepad2.dpad_down;
    }

    protected boolean isAButtonPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return gamepad1.a || gamepad2.a;
    }
}
