package org.firstinspires.ftc.teamcode.utils.general;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.general.prompts.Prompt;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ChoiceMenu {
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private final List<Prompt> prompts = new ArrayList<>();
    private final Map<String, Object> results = new HashMap<>();

    private int currentIndex = 0;

    public ChoiceMenu(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Add a prompt to the queue.
     */
    public void enqueuePrompt(Prompt prompt) {
        prompts.add(prompt);
    }

    /**
     * Gets the chosen value of a prompt from its key. Call this in the start function.
     *
     * @param key The prompt's key
     * @return The Object value of the prompt result
     */
    public Object getValueOf(String key) {
        return results.get(key);
    }

    /**
     * Handles the prompts and inputs. Call this in the init_loop function.
     */
    public void processPrompts() {
        // Handle back navigation
        if (Debounce.isButtonPressed("b", gamepad1.b, gamepad2.b) && currentIndex > 0) {
            currentIndex--;
        }

        // Display results if no prompts left
        if (currentIndex >= prompts.size()) {
            for (Map.Entry<String, Object> entry : results.entrySet()) {
                telemetry.addData(entry.getKey(), entry.getValue());
            }
            return;
        }

        // Get the current prompt
        Prompt currentPrompt = prompts.get(currentIndex);

        // Process prompt
        Object result = currentPrompt.process(gamepad1, gamepad2, telemetry);

        if (result != null) {
            results.put(currentPrompt.getKey(), result);
            currentIndex++;
        }
    }
}
