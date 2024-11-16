package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.MotionProfileConfig;

@TeleOp(name = "Motion Profiling Test", group = "tests")
public class MotionProfiling extends OpMode {
    private double previousAxialSpeed = 0;
    private double previousLateralSpeed = 0;
    private double previousYawSpeed = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private MecanumDrive drive;

    private double lastTime = 0;
    private boolean isJoystickActive = false;

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
     * Parabolic smoothing method with time adjustment
     */
    private double applyParabolicSmoothing(double targetSpeed, double previousSpeed, double beta, int power) {
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
        int power = MotionProfileConfig.PARABOLIC_SMOOTHING_POWER;

        // Apply parabolic smoothing to each multiplier
        double smoothedLateralSpeed = applyParabolicSmoothing(
                -gamepad1.left_stick_x * MotionProfileConfig.LATERAL_MULTIPLIER,
                previousLateralSpeed,
                beta,
                power
        );

        double smoothedAxialSpeed = applyParabolicSmoothing(
                -gamepad1.left_stick_y * MotionProfileConfig.AXIAL_MULTIPLIER,
                previousAxialSpeed,
                beta,
                power
        );

        double smoothedYawSpeed = applyParabolicSmoothing(
                -gamepad1.right_stick_x * MotionProfileConfig.YAW_MULTIPLIER,
                previousYawSpeed,
                beta,
                power
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

    /**
     * Global joystick movement with ramp-up and deceleration.
     */
    public void globalJoystickMovement() {
        double currentTime = timer.seconds();

        // Joystick inputs
        double lateralInput = -gamepad1.left_stick_x;
        double axialInput = -gamepad1.left_stick_y;
        double yawInput = -gamepad1.right_stick_x;

        // Determine if joystick is active
        boolean isActive = Math.abs(lateralInput) > 0.01 || Math.abs(axialInput) > 0.01 || Math.abs(yawInput) > 0.01;

        if (isActive) {
            if (!isJoystickActive) {
                // Transition from inactive to active: ramp-up
                previousAxialSpeed = applySlewRate(axialInput * MotionProfileConfig.AXIAL_MULTIPLIER, previousAxialSpeed, lastTime);
                previousLateralSpeed = applySlewRate(lateralInput * MotionProfileConfig.LATERAL_MULTIPLIER, previousLateralSpeed, lastTime);
                previousYawSpeed = applySlewRate(yawInput * MotionProfileConfig.YAW_MULTIPLIER, previousYawSpeed, lastTime);
            } else {
                // Maintain speed during joystick movement
                previousAxialSpeed = axialInput * MotionProfileConfig.AXIAL_MULTIPLIER;
                previousLateralSpeed = lateralInput * MotionProfileConfig.LATERAL_MULTIPLIER;
                previousYawSpeed = yawInput * MotionProfileConfig.YAW_MULTIPLIER;
            }
        } else if (isJoystickActive) {
            // Deceleration when joystick is released
            previousAxialSpeed = applySlewRate(0, previousAxialSpeed, lastTime);
            previousLateralSpeed = applySlewRate(0, previousLateralSpeed, lastTime);
            previousYawSpeed = applySlewRate(0, previousYawSpeed, lastTime);
        }

        // Update joystick state
        isJoystickActive = isActive;

        // Apply calculated speeds to the drive
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(previousAxialSpeed, previousLateralSpeed),
                        previousYawSpeed
                )
        );

        drive.updatePoseEstimate();
        lastTime = currentTime;

        telemetry.addData("Axial Speed", previousAxialSpeed);
        telemetry.addData("Lateral Speed", previousLateralSpeed);
        telemetry.addData("Yaw Speed", previousYawSpeed);
    }

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        //slewRateMovement();
        //exponentialSmoothingMovement();
        parabolicSmoothingMovement();
        //globalJoystickMovement();

        telemetry.addData("time: ", timer.seconds());
        telemetry.update();
    }
}
