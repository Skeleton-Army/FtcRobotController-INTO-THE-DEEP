package org.firstinspires.ftc.teamcode.utils.config;

import com.skeletonarmy.marrow.MarrowGamepad;

public class OneDriverTeleopControls extends TeleopControls {
    public OneDriverTeleopControls(MarrowGamepad gamepad1, MarrowGamepad gamepad2) {
        super(gamepad1, gamepad2);
        
        SLOW_MODE_BUTTON = gamepad2.right_bumper;

        INTAKE_BUTTON = gamepad1.a;
        INTAKE_ROTATION = gamepad1.left_bumper;
        INTAKE_MANUAL_Y = gamepad2.right_stick_y;

        OUTTAKE_BUTTON = gamepad1.y;
        OUTTAKE_CYCLE_BUTTON = gamepad1.back;

        GRAB_BUTTON = gamepad1.right_bumper;
        RELEASE_BUTTON = gamepad2.left_bumper;

        SPECIMEN_HANG_BUTTON = gamepad1.left_trigger;
        SPECIMEN_INTAKE_BUTTON = gamepad1.right_trigger;
        SPECIMEN_MANUAL_Y = gamepad2.right_stick_y;

        HANG_BUTTON = gamepad1.start;

        EMERGENCY_STOP_BUTTON = gamepad1.b;
        RESET_MOTORS_BUTTON = gamepad1.guide;
    }
}
