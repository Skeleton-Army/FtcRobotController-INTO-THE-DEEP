package org.firstinspires.ftc.teamcode.utils.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.config.MotionProfileConfig;

public class MotionProfiling {
    private static final ElapsedTime timer = new ElapsedTime();

    private static double previousAxialSpeed = 0;
    private static double previousLateralSpeed = 0;
    private static double previousYawSpeed = 0;
    private static double lastTime = 0;

    /**
     * Parabolic smoothing method with time adjustment
     */
    private static double applySmoothing(double targetSpeed, double previousSpeed, double beta, double power) {
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;

        // Calculate the distance (difference) between the target and the current speed
        double speedDifference = targetSpeed - previousSpeed;

        // Apply the parabolic formula
        double parabolicChange = beta * Math.pow(speedDifference, power * 2) * Math.signum(speedDifference) * deltaTime;

        // Blend with a small linear adjustment to prevent stalling for small differences
        double linearChange = 0.1 * speedDifference;

        // Final change is a combination of linear and parabolic smoothing
        double totalChange = linearChange + parabolicChange;

        // Clamp the result to avoid overshooting the target speed
        if (Math.abs(targetSpeed - (previousSpeed + totalChange)) < Math.abs(speedDifference)) {
            return previousSpeed + totalChange;
        } else {
            return targetSpeed;
        }
    }

    public static double calculateSmoothedYawSpeed(Gamepad gamepad) {
        double beta = MotionProfileConfig.PARABOLIC_SMOOTHING_BETA;
        double power = MotionProfileConfig.PARABOLIC_SMOOTHING_POWER;

        double smoothedYawSpeed = applySmoothing(
                -gamepad.right_stick_x * MotionProfileConfig.YAW_MULTIPLIER,
                previousYawSpeed,
                beta,
                power
        );

        previousYawSpeed = smoothedYawSpeed;
        return smoothedYawSpeed;
    }

    public static Vector2d getSmoothingPowersVector2D(Gamepad gamepad) {
        Vector2d v = getSmoothingPowersVelPose(gamepad).component1();
        return v;
    }

    public static PoseVelocity2d getSmoothingPowersVelPose(Gamepad gamepad) {
        double beta = MotionProfileConfig.PARABOLIC_SMOOTHING_BETA;
        double power = MotionProfileConfig.PARABOLIC_SMOOTHING_POWER;

        // Apply parabolic smoothing to each multiplier
        double smoothedAxialSpeed = applySmoothing(
                gamepad.left_stick_y * MotionProfileConfig.AXIAL_MULTIPLIER,
                previousAxialSpeed,
                beta,
                power
        );

        double smoothedLateralSpeed = applySmoothing(
                -gamepad.left_stick_x * MotionProfileConfig.LATERAL_MULTIPLIER,
                previousLateralSpeed,
                beta,
                power
        );

        double smoothedYawSpeed = calculateSmoothedYawSpeed(gamepad);

        // Update previous speeds for the next cycle
        previousAxialSpeed = smoothedAxialSpeed;
        previousLateralSpeed = smoothedLateralSpeed;
        previousYawSpeed = smoothedYawSpeed;

        lastTime = timer.seconds();

        return new PoseVelocity2d(
                new Vector2d(
                        smoothedAxialSpeed,
                        smoothedLateralSpeed
                ),
                smoothedYawSpeed
        );
    }
}
