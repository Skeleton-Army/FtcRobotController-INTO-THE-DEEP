package org.firstinspires.ftc.teamcode.utils.general.prompts;

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
}
