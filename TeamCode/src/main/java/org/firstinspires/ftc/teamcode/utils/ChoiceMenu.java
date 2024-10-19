package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.prompts.Prompt;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;

public class ChoiceMenu {
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private final Queue<Prompt> promptQueue = new LinkedList<>();
    private final Map<String, Object> results = new HashMap<>();

    private Prompt currentPrompt = null;

    public ChoiceMenu(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Add a prompt to the queue.
     */
    public void enqueuePrompt(Prompt prompt) {
        promptQueue.add(prompt);
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
        if (currentPrompt == null && promptQueue.isEmpty()) {
            for (Map.Entry<String, Object> entry : results.entrySet()) {
                telemetry.addData(entry.getKey(), entry.getValue());
            }

            return;
        }

        // Get the next prompt
        if (currentPrompt == null) {
            currentPrompt = promptQueue.poll();
        }

        // Process prompt
        if (currentPrompt != null) {
            Object result = currentPrompt.process(gamepad1, gamepad2, telemetry);

            if (result != null) {
                results.put(currentPrompt.getKey(), result);
                currentPrompt = null;
            }
        }
    }
}
