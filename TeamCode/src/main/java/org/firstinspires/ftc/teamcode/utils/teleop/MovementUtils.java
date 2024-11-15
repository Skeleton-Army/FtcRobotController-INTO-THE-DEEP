package org.firstinspires.ftc.teamcode.utils.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opModes.TeleopApplication;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;

public class MovementUtils {
    final double SLOW_MODE_MULTIPLIER = 0.3;

    double multiplier;

    MecanumDrive drive;

    public MovementUtils(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
    }

    void calculateMultipliers() {
        boolean slowModeActive = TeleopApplication.Instance.gamepad1.right_bumper || TeleopApplication.Instance.gamepad2.start;

        multiplier = slowModeActive ? SLOW_MODE_MULTIPLIER : 1;
    }

    public void movement(Gamepad gamepad) {
        calculateMultipliers();

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(-gamepad.left_stick_y * multiplier, -gamepad.left_stick_x * multiplier),
                        -gamepad.right_stick_x * multiplier
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

        Vector2d input = Utilities.rotate(new Vector2d(
                -TeleopApplication.Instance.gamepad1.left_stick_y,
                -TeleopApplication.Instance.gamepad1.left_stick_x
        ), -poseEstimate.heading.toDouble());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(input.x * multiplier, input.y * multiplier),
                        -TeleopApplication.Instance.gamepad1.right_stick_x * multiplier
                )
        );

        // Update everything. Odometry. Etc.
        drive.updatePoseEstimate();
    }
}