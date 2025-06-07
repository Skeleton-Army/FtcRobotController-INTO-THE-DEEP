package org.firstinspires.ftc.teamcode.utils.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.skeletonarmy.marrow.MarrowGamepad;

import org.firstinspires.ftc.teamcode.opModes.TeleopApplication;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.MotionProfileConfig;

public class MovementUtils {
    MecanumDrive drive;

    Gamepad gamepad;
    MarrowGamepad.ControlState<?> slowModeButton;

    double multiplier;

    public MovementUtils(Gamepad gamepad, MarrowGamepad.ControlState<?> slowModeButton) {
        drive = TeleopApplication.Instance.drive;

        this.gamepad = gamepad;
        this.slowModeButton = slowModeButton;
    }

    void calculateMultipliers() {
        boolean slowModeActive = slowModeButton.isDown();

        multiplier = slowModeActive ? MotionProfileConfig.SLOW_MODE_MULTIPLIER : 1;
    }

    public void movement() {
        calculateMultipliers();

        // Get the smoothed velocity and pose
        PoseVelocity2d smoothedVelPose = MotionProfiling.getSmoothingPowersVelPose(gamepad);

        // Apply the slow mode multiplier to the input
        PoseVelocity2d velPoseWithMultiplier = new PoseVelocity2d(
                new Vector2d(
                        smoothedVelPose.linearVel.x * multiplier,
                        smoothedVelPose.linearVel.y * multiplier
                ),
                smoothedVelPose.angVel * multiplier
        );

        drive.setDrivePowers(velPoseWithMultiplier);

        drive.updatePoseEstimate();
    }

    public void fieldCentricMovement() {
        calculateMultipliers();

        // Read pose
        Pose2d poseEstimate = drive.pose;

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = rotate((MotionProfiling.getSmoothingPowersVector2D(gamepad)
        ), -drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(input.x * multiplier, input.y * multiplier),
                        MotionProfiling.calculateSmoothedYawSpeed(gamepad) * multiplier
                )
        );

        // Update everything. Odometry. Etc.
        drive.updatePoseEstimate();
    }

    public static Vector2d rotate(Vector2d original, double angle) {
        double rx = (original.x * Math.cos(angle)) - (original.y * Math.sin(angle));
        double ry = (original.x * Math.sin(angle)) + (original.y * Math.cos(angle));

        return new Vector2d(rx, ry);
    }
}