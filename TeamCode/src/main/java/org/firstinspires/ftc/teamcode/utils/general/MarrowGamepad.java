package org.firstinspires.ftc.teamcode.utils.general;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;

public class MarrowGamepad {
    private final OpMode opMode;

    private final Gamepad current;
    private final Gamepad previous = new Gamepad();
    private final Gamepad snapshot = new Gamepad();

    private double lastOpModeTime = -1;

    // Gamepad Buttons
    public final ButtonState a;
    public final ButtonState b;
    public final ButtonState x;
    public final ButtonState y;
    public final ButtonState dpad_up;
    public final ButtonState dpad_down;
    public final ButtonState dpad_left;
    public final ButtonState dpad_right;
    public final ButtonState left_bumper;
    public final ButtonState right_bumper;
    public final ButtonState left_stick_button;
    public final ButtonState right_stick_button;
    public final ButtonState start;
    public final ButtonState back;
    public final ButtonState guide;

    public final AnalogState left_trigger;
    public final AnalogState right_trigger;

    public final SimpleAnalogState left_stick_x;
    public final SimpleAnalogState left_stick_y;
    public final SimpleAnalogState right_stick_x;
    public final SimpleAnalogState right_stick_y;

    // PS4 Support
    public final ButtonState circle;
    public final ButtonState cross;
    public final ButtonState triangle;
    public final ButtonState square;
    public final ButtonState share;
    public final ButtonState options;
    public final ButtonState touchpad;
    public final ButtonState ps;

    // PS4 Touchpad
    public final ButtonState touchpad_finger_1;
    public final ButtonState touchpad_finger_2;

    public final SimpleAnalogState touchpad_finger_1_x;
    public final SimpleAnalogState touchpad_finger_1_y;
    public final SimpleAnalogState touchpad_finger_2_x;
    public final SimpleAnalogState touchpad_finger_2_y;

    public MarrowGamepad(OpMode opMode, Gamepad gamepad) {
        this.opMode = opMode;
        this.current = gamepad;

        // Gamepad Buttons
        a = new ButtonState(() -> current.a, () -> previous.a);
        b = new ButtonState(() -> current.b, () -> previous.b);
        x = new ButtonState(() -> current.x, () -> previous.x);
        y = new ButtonState(() -> current.y, () -> previous.y);
        dpad_up = new ButtonState(() -> current.dpad_up, () -> previous.dpad_up);
        dpad_down = new ButtonState(() -> current.dpad_down, () -> previous.dpad_down);
        dpad_left = new ButtonState(() -> current.dpad_left, () -> previous.dpad_left);
        dpad_right = new ButtonState(() -> current.dpad_right, () -> previous.dpad_right);
        left_bumper = new ButtonState(() -> current.left_bumper, () -> previous.left_bumper);
        right_bumper = new ButtonState(() -> current.right_bumper, () -> previous.right_bumper);
        left_stick_button = new ButtonState(() -> current.left_stick_button, () -> previous.left_stick_button);
        right_stick_button = new ButtonState(() -> current.right_stick_button, () -> previous.right_stick_button);
        start = new ButtonState(() -> current.start, () -> previous.start);
        back = new ButtonState(() -> current.back, () -> previous.back);
        guide = new ButtonState(() -> current.guide, () -> previous.guide);

        left_trigger = new AnalogState(() -> current.left_trigger, () -> previous.left_trigger);
        right_trigger = new AnalogState(() -> current.right_trigger, () -> previous.right_trigger);

        left_stick_x = new SimpleAnalogState(() -> current.left_stick_x);
        left_stick_y = new SimpleAnalogState(() -> current.left_stick_y);
        right_stick_x = new SimpleAnalogState(() -> current.right_stick_x);
        right_stick_y = new SimpleAnalogState(() -> current.right_stick_y);

        // PS4 Buttons
        circle = new ButtonState(() -> current.circle, () -> previous.circle);
        cross = new ButtonState(() -> current.cross, () -> previous.cross);
        triangle = new ButtonState(() -> current.triangle, () -> previous.triangle);
        square = new ButtonState(() -> current.square, () -> previous.square);
        share = new ButtonState(() -> current.share, () -> previous.share);
        options = new ButtonState(() -> current.options, () -> previous.options);
        touchpad = new ButtonState(() -> current.touchpad, () -> previous.touchpad);
        ps = new ButtonState(() -> current.ps, () -> previous.ps);

        // PS4 Touchpad
        touchpad_finger_1 = new ButtonState(() -> current.touchpad_finger_1, () -> previous.touchpad_finger_1);
        touchpad_finger_2 = new ButtonState(() -> current.touchpad_finger_2, () -> previous.touchpad_finger_2);

        touchpad_finger_1_x = new SimpleAnalogState(() -> current.touchpad_finger_1_x);
        touchpad_finger_1_y = new SimpleAnalogState(() -> current.touchpad_finger_1_y);
        touchpad_finger_2_x = new SimpleAnalogState(() -> current.touchpad_finger_2_x);
        touchpad_finger_2_y = new SimpleAnalogState(() -> current.touchpad_finger_2_y);
    }

    /**
     * Updates the internal gamepad state.
     */
    public void update() {
        if (opMode.time == lastOpModeTime) return; // Skip update if it was already called this frame. This is so it runs only once per loop.

        // Save snapshot (last frame) into previous
        previous.copy(snapshot);

        // Save current into snapshot
        snapshot.copy(current);

        lastOpModeTime = opMode.time;
    }

    /**
     * Returns the original (vanilla) gamepad.
     *
     * @return the current {@link Gamepad} instance
     */
    public Gamepad gamepad() {
        return current;
    }

    // --- Base class ---
    public abstract class ControlState<T> {
        protected Supplier<T> current;
        protected Supplier<T> previous;

        private boolean toggled;
        private long holdStartTime;
        private boolean justHeldTriggered = false;

        public ControlState(Supplier<T> current, Supplier<T> previous) {
            this.current = current;
            this.previous = previous;
            this.toggled = false;
            this.holdStartTime = -1;
        }

        protected void updateHoldTime() {
            if (isDown()) {
                if (holdStartTime == -1) {
                    holdStartTime = System.nanoTime();
                    justHeldTriggered = false;
                }
            } else {
                holdStartTime = -1;
                justHeldTriggered = false;
            }
        }

        protected T value() {
            return current.get();
        }

        public abstract boolean isDown();
        public abstract boolean isUp();
        public abstract boolean isJustPressed();
        public abstract boolean isJustReleased();

        public boolean isToggled() {
            if (isJustPressed()) {
                toggled = !toggled;
            }
            return toggled;
        }

        public boolean isHeld(double durationInSeconds) {
            updateHoldTime();

            if (holdStartTime == -1) {
                return false;
            }

            long elapsedTimeMillis = (System.nanoTime() - holdStartTime) / 1_000_000;
            long elapsedTimeSeconds = elapsedTimeMillis / 1000;

            return elapsedTimeSeconds >= durationInSeconds;
        }

        public boolean isJustHeld(double durationInSeconds) {
            updateHoldTime();

            if (holdStartTime == -1 || justHeldTriggered) {
                return false;
            }

            long elapsedTimeMillis = (System.nanoTime() - holdStartTime) / 1_000_000;
            long elapsedTimeSeconds = elapsedTimeMillis / 1000;

            if (elapsedTimeSeconds >= durationInSeconds) {
                justHeldTriggered = true;
                return true;
            }

            return false;
        }
    }

    public class ButtonState extends ControlState<Boolean> {
        public ButtonState(Supplier<Boolean> current, Supplier<Boolean> previous) {
            super(current, previous);
        }

        @Override
        public boolean isDown() {
            return value();
        }

        @Override
        public boolean isUp() {
            return !value();
        }

        @Override
        public boolean isJustPressed() {
            update();
            return !previous.get() && isDown();
        }

        @Override
        public boolean isJustReleased() {
            update();
            return previous.get() && isUp();
        }
    }

    public class AnalogState extends ControlState<Float> {
        private final float threshold;

        public AnalogState(Supplier<Float> current, Supplier<Float> previous) {
            this(current, previous, 0.1f);
        }

        public AnalogState(Supplier<Float> current, Supplier<Float> previous, float threshold) {
            super(current, previous);
            this.threshold = threshold;
        }

        @Override
        public Float value() {
            return current.get();
        }

        @Override
        public boolean isDown() {
            return value() >= threshold;
        }

        @Override
        public boolean isUp() {
            return value() < threshold;
        }

        @Override
        public boolean isJustPressed() {
            update();
            return previous.get() < threshold && isDown();
        }

        @Override
        public boolean isJustReleased() {
            update();
            return previous.get() >= threshold && isUp();
        }
    }

    public class SimpleAnalogState {
        protected Supplier<Float> current;

        public SimpleAnalogState(Supplier<Float> current) {
            this.current = current;
        }

        public float value() {
            return current.get();
        }
    }
}
