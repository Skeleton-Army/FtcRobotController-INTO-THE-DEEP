package org.firstinspires.ftc.teamcode.utils.general;

import com.acmerobotics.dashboard.config.Config;

import java.util.HashMap;

@Config
public class Debounce {
    public static long DEBOUNCE_DELAY = 500; // in milliseconds

    private static final HashMap<String, Long> lastPressTimes = new HashMap<>();

    /**
     * Checks if a button was pressed while handling debouncing for the specific button.
     *
     * @param buttonName The name of the button (used to track debouncing separately).
     * @param button     The current state of the button (true if pressed, false otherwise).
     * @return True if the button is pressed and not within the debounce delay, false otherwise.
     */
    public static boolean isButtonPressed(String buttonName, boolean button) {
        long currentTime = System.currentTimeMillis();

        // Handle debouncing for the specific button
        if (button) {
            long lastPressTime = lastPressTimes.getOrDefault(buttonName, 0L);
            if (currentTime - lastPressTime >= DEBOUNCE_DELAY) {
                lastPressTimes.put(buttonName, currentTime);
                return true;
            }
        }

        return false;
    }

    /**
     * Checks if any of the specified buttons were pressed while handling debouncing for each.
     *
     * @param buttonName  The name of the button.
     * @param button1     The current state of the first button (true if pressed, false otherwise).
     * @param button2     The current state of the second button (true if pressed, false otherwise).
     * @return True if any of the buttons are pressed and not within the debounce delay, false otherwise.
     */
    public static boolean isButtonPressed(String buttonName, boolean button1, boolean button2) {
        return isButtonPressed(buttonName, button1 || button2);
    }
}
