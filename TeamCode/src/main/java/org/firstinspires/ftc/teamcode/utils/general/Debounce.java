package org.firstinspires.ftc.teamcode.utils.general;

public class Debounce {
    private static final long DEBOUNCE_DELAY = 500; // in milliseconds

    private static long lastPressTime = 0;

    public static boolean isButtonPressed(boolean button) {
        long currentTime = System.currentTimeMillis();

        // Handle debouncing
        if (button) {
            if (currentTime - lastPressTime >= DEBOUNCE_DELAY) {
                lastPressTime = currentTime;
                return true;
            }
        }

        return false;
    }

    public static boolean isButtonPressed(boolean button1, boolean button2) {
        return isButtonPressed(button1 || button2);
    }
}
