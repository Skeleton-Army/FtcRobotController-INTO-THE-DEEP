package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ChoiceMenuInput {
    private static final long DEBOUNCE_DELAY = 500; // in milliseconds

    private static long lastPressTime = 0;

    private static boolean isButtonPressed(boolean button1, boolean button2) {
        long currentTime = System.currentTimeMillis();
        boolean pressed = button1 || button2;

        // Handle debouncing
        if (pressed) {
            if (currentTime - lastPressTime >= DEBOUNCE_DELAY) {
                lastPressTime = currentTime;
                return true;
            }
        }

        return false;
    }

    public static boolean isDpadLeftPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return isButtonPressed(gamepad1.dpad_left, gamepad2.dpad_left);
    }

    public static boolean isDpadRightPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return isButtonPressed(gamepad1.dpad_right, gamepad2.dpad_right);
    }

    public static boolean isDpadUpPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return isButtonPressed(gamepad1.dpad_up, gamepad2.dpad_up);
    }

    public static boolean isDpadDownPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return isButtonPressed(gamepad1.dpad_down, gamepad2.dpad_down);
    }

    public static boolean isAButtonPressed(Gamepad gamepad1, Gamepad gamepad2) {
        return isButtonPressed(gamepad1.a, gamepad2.a);
    }
}
