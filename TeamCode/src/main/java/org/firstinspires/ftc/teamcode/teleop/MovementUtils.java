package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class MovementUtils {
    final double SLOW_MODE_MULTIPLIER = 0.3;

    double axialMultiplier;
    double lateralMultiplier;
    double yawMultiplier;

    MecanumDrive drive;

    public MovementUtils(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    }

    void calculateMultipliers() {
        boolean slowModeActive = Controller.Instance.gamepad1.right_bumper || Controller.Instance.gamepad2.start;

        axialMultiplier = slowModeActive ? SLOW_MODE_MULTIPLIER : 1;
        lateralMultiplier = slowModeActive ? SLOW_MODE_MULTIPLIER : 1;
        yawMultiplier = slowModeActive ? SLOW_MODE_MULTIPLIER : 1;
    }

    public void movement(Gamepad gamepad) {
        calculateMultipliers();

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(-gamepad.left_stick_y * axialMultiplier, -gamepad.left_stick_x * lateralMultiplier),
                        -gamepad.right_stick_x * yawMultiplier
                )
        );

        drive.updatePoseEstimate();
    }

    public void fieldCentricMovement() {
        calculateMultipliers();

        // Read pose
        Pose2d poseEstimate = drive.pose;

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading

        Vector2d input = new Vector2d(
                -Controller.Instance.gamepad1.left_stick_y,
                -Controller.Instance.gamepad1.left_stick_x
        );

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(input.x * axialMultiplier, input.y * lateralMultiplier),
                        -Controller.Instance.gamepad1.right_stick_x * yawMultiplier
                )
        );

        // Update everything. Odometry. Etc.
        drive.updatePoseEstimate();
    }
}