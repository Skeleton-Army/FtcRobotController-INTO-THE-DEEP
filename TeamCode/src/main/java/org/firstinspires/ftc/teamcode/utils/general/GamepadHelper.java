package org.firstinspires.ftc.teamcode.utils.general;

import java.util.HashMap;

public class GamepadHelper {
    private static final HashMap<Integer, Boolean> lastButtonStates = new HashMap<>();

    /**
     * Checks if a button was pressed (i.e., transitioned from false to true).
     *
     * @param button The current state of the button (true if pressed, false otherwise).
     * @return True if the button transitioned from false to true, false otherwise.
     */
    public static boolean isButtonPressed(boolean button) {
        int uniqueKey = getCallSiteHash();

        // Get the previous state (default to false if not tracked yet)
        boolean lastState = Boolean.TRUE.equals(lastButtonStates.getOrDefault(uniqueKey, false));

        // Update the stored state
        lastButtonStates.put(uniqueKey, button);

        // Return true if the last state was false and the current state is true (button just pressed)
        return !lastState && button;
    }

    /**
     * Generates a unique key based on the calling location.
     *
     * @return A hash code representing the call site.
     */
    public static int getCallSiteHash() {
        StackTraceElement caller = Thread.currentThread().getStackTrace()[2];
        return caller.hashCode(); // Generates a consistent identifier for each call site
    }
}
