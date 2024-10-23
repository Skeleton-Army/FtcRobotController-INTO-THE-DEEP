package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class MovementUtils {
    final double AXIAL_SPEED = 1;
    final double LATERAL_SPEED = 1;
    final double YAW_SPEED = 1;
    final double SLOW_MODE_MULTIPLIER = 0.3;

    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    double axialMultiplier;
    double lateralMultiplier;
    double yawMultiplier;

    MecanumDrive drive;
    Vector2d targetPosition = new Vector2d(0, 0);

    public MovementUtils(HardwareMap hardwareMap) {
        // Initialize SampleMecanumDrive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    }

    void calculateMultipliers(Gamepad gamepad) {
        boolean slowModeActive = gamepad.right_bumper || Controller.Instance.gamepad2.start;

        axialMultiplier = AXIAL_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
        lateralMultiplier = LATERAL_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
        yawMultiplier = YAW_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
    }

    public void movement(Gamepad gamepad) {
        calculateMultipliers(gamepad);

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(-gamepad.left_stick_y * axialMultiplier, -gamepad.left_stick_x * lateralMultiplier),
                        -gamepad.right_stick_x * yawMultiplier
                )
        );

        drive.updatePoseEstimate();
    }

    public void fieldCentricMovement(Gamepad gamepad, Telemetry telemetry) {
        calculateMultipliers(gamepad);

        // Read pose
        Pose2d poseEstimate = drive.pose;

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading


        // TODO: how should we do it?
        /*
        Vector2d input = new Vector2d(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x
        ).rotated(-poseEstimate.heading);
         */

        Vector2d input = new Vector2d(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x
        );

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(input.x * axialMultiplier, input.y * lateralMultiplier),
                        -gamepad.right_stick_x * yawMultiplier
                )
        );

        // Update everything. Odometry. Etc.
        drive.updatePoseEstimate();
    }
}