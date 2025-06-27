package org.firstinspires.ftc.teamcode.utils.config;

import org.firstinspires.ftc.teamcode.utils.general.MarrowGamepad;

public class TeleopControls {
    public MarrowGamepad.ControlState<?> SLOW_MODE_BUTTON;

    public MarrowGamepad.ControlState<?> INTAKE_BUTTON;
    public MarrowGamepad.ControlState<?> INTAKE_THROW_BUTTON;
    public MarrowGamepad.SimpleAnalogState INTAKE_ROTATION_X;
    public MarrowGamepad.SimpleAnalogState INTAKE_ROTATION_Y;
    public MarrowGamepad.SimpleAnalogState INTAKE_MANUAL_Y;

    public MarrowGamepad.ControlState<?> INTAKE_ROTATION;

    public MarrowGamepad.ControlState<?> OUTTAKE_BUTTON;
    public MarrowGamepad.ControlState<?> OUTTAKE_CYCLE_BUTTON;

    public MarrowGamepad.ControlState<?> GRAB_BUTTON;
    public MarrowGamepad.ControlState<?> RELEASE_BUTTON;

    public MarrowGamepad.ControlState<?> SPECIMEN_HANG_BUTTON;
    public MarrowGamepad.ControlState<?> SPECIMEN_INTAKE_BUTTON;
    public MarrowGamepad.SimpleAnalogState SPECIMEN_MANUAL_Y;

    public MarrowGamepad.ControlState<?> HANG_BUTTON;

    public MarrowGamepad.ControlState<?> EMERGENCY_STOP_BUTTON;
    public MarrowGamepad.ControlState<?> RESET_MOTORS_BUTTON;

    public TeleopControls(MarrowGamepad gamepad1, MarrowGamepad gamepad2) {
        SLOW_MODE_BUTTON = gamepad1.right_bumper;

        INTAKE_BUTTON = gamepad2.a;
        INTAKE_THROW_BUTTON = gamepad2.a;
        INTAKE_ROTATION_X = gamepad2.left_stick_x;
        INTAKE_ROTATION_Y = gamepad2.left_stick_y;
        INTAKE_MANUAL_Y = gamepad2.right_stick_y;

        OUTTAKE_BUTTON = gamepad2.y;
        OUTTAKE_CYCLE_BUTTON = gamepad2.share;

        GRAB_BUTTON = gamepad2.right_bumper;
        RELEASE_BUTTON = gamepad2.left_bumper;

        SPECIMEN_HANG_BUTTON = gamepad2.dpad_up;
        SPECIMEN_INTAKE_BUTTON = gamepad2.dpad_down;
        SPECIMEN_MANUAL_Y = gamepad2.right_stick_y;

        HANG_BUTTON = gamepad2.start;

        EMERGENCY_STOP_BUTTON = gamepad2.b;
        RESET_MOTORS_BUTTON = gamepad2.guide;
    }
}
