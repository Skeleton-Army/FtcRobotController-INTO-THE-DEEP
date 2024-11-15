package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.MotionProfileConfig;

@TeleOp(name="Motion Profiling Test", group = "tests")
public class MotionProfiling extends OpMode {
    private double previousAxialSpeed = 0;
    private double previousLateralSpeed = 0;
    private double previousYawSpeed = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private MecanumDrive drive;

    private double lastTime = 0;

    /**
     * Linear slew rate limiter
     */
    private double applySlewRate(double targetSpeed, double previousSpeed, double calledTime) {
        double currentTime = timer.seconds();
        double deltaTime = currentTime - calledTime;

        telemetry.addData("delta time: ", deltaTime);
        double maxChange = MotionProfileConfig.MAX_SLEW_RATE * deltaTime;
        double speedChange = targetSpeed - previousSpeed;

        if (Math.abs(speedChange) > maxChange) {
            speedChange = Math.signum(speedChange) * maxChange;
        }

        return previousSpeed + speedChange;
    }

    /**
     * Exponential smoothing method
     */
    private double applyExponentialSmoothing(double targetSpeed, double previousSpeed, double alpha) {
        return previousSpeed + alpha * (targetSpeed - previousSpeed);
    }

    /**
     * Parabolic smoothing method
     */
    private double applyParabolicSmoothing(double targetSpeed, double previousSpeed, double beta) {
        // Calculate the distance (difference) between the target and the current speed
        double speedDifference = targetSpeed - previousSpeed;

        // Apply the parabolic formula
        double change = beta * Math.pow(speedDifference, 2) * Math.signum(speedDifference);

        // Return the updated speed
        return previousSpeed + change;
    }

    public void slewRateMovement() {
        // Apply slew rate limiter to each multiplier
        double limitedLateralSpeed = applySlewRate(-gamepad1.left_stick_x * MotionProfileConfig.LATERAL_MULTIPLIER, previousLateralSpeed, lastTime);
        double limitedAxialSpeed = applySlewRate(-gamepad1.left_stick_y * MotionProfileConfig.AXIAL_MULTIPLIER, previousAxialSpeed, lastTime);
        double limitedYawSpeed = applySlewRate(-gamepad1.right_stick_x * MotionProfileConfig.YAW_MULTIPLIER, previousYawSpeed, lastTime);

        // Update previous speeds for the next cycle
        previousAxialSpeed = limitedAxialSpeed;
        previousLateralSpeed = limitedLateralSpeed;
        previousYawSpeed = limitedYawSpeed;

        lastTime = timer.seconds();

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                limitedAxialSpeed,
                                limitedLateralSpeed
                        ),
                        limitedYawSpeed
                )
        );

        drive.updatePoseEstimate();
    }

    public void exponentialSmoothingMovement() {
        double alpha = MotionProfileConfig.EXPONENTIAL_SMOOTHING_ALPHA;

        // Apply exponential smoothing to each multiplier
        double smoothedLateralSpeed = applyExponentialSmoothing(
                -gamepad1.left_stick_x * MotionProfileConfig.LATERAL_MULTIPLIER,
                previousLateralSpeed,
                alpha
        );

        double smoothedAxialSpeed = applyExponentialSmoothing(
                -gamepad1.left_stick_y * MotionProfileConfig.AXIAL_MULTIPLIER,
                previousAxialSpeed,
                alpha
        );

        double smoothedYawSpeed = applyExponentialSmoothing(
                -gamepad1.right_stick_x * MotionProfileConfig.YAW_MULTIPLIER,
                previousYawSpeed,
                alpha
        );

        // Update previous speeds for the next cycle
        previousAxialSpeed = smoothedAxialSpeed;
        previousLateralSpeed = smoothedLateralSpeed;
        previousYawSpeed = smoothedYawSpeed;

        lastTime = timer.seconds();

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                smoothedAxialSpeed,
                                smoothedLateralSpeed
                        ),
                        smoothedYawSpeed
                )
        );

        drive.updatePoseEstimate();
    }

    public void parabolicSmoothingMovement() {
        double beta = MotionProfileConfig.PARABOLIC_SMOOTHING_BETA;

        // Apply parabolic smoothing to each multiplier
        double smoothedLateralSpeed = applyParabolicSmoothing(
                -gamepad1.left_stick_x * MotionProfileConfig.LATERAL_MULTIPLIER,
                previousLateralSpeed,
                beta
        );

        double smoothedAxialSpeed = applyParabolicSmoothing(
                -gamepad1.left_stick_y * MotionProfileConfig.AXIAL_MULTIPLIER,
                previousAxialSpeed,
                beta
        );

        double smoothedYawSpeed = applyParabolicSmoothing(
                -gamepad1.right_stick_x * MotionProfileConfig.YAW_MULTIPLIER,
                previousYawSpeed,
                beta
        );

        // Update previous speeds for the next cycle
        previousAxialSpeed = smoothedAxialSpeed;
        previousLateralSpeed = smoothedLateralSpeed;
        previousYawSpeed = smoothedYawSpeed;

        lastTime = timer.seconds();

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                smoothedAxialSpeed,
                                smoothedLateralSpeed
                        ),
                        smoothedYawSpeed
                )
        );

        drive.updatePoseEstimate();
    }

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        //slewRateMovement();
        exponentialSmoothingMovement();
        //parabolicSmoothingMovement();

        telemetry.addData("time: ", timer.seconds());
        telemetry.update();
    }
}
