package org.firstinspires.ftc.teamcode.teleop.motionprofile;

import static org.firstinspires.ftc.teamcode.teleop.motionprofile.MotionProfilingConstants.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="MotionProfilingTest", group = "tests")
public class MotionProfiling extends OpMode {
    private double previousAxialSpeed = 0;
    private double previousLateralSpeed = 0;
    private double previousYawSpeed = 0;
    private ElapsedTime timer = new ElapsedTime();
    private MecanumDrive drive;

    private double lastTime = 0;

    double currentTime = timer.seconds();


    private double applySlewRate(double targetSpeed, double previousSpeed, double calledTime) {

        //timer.reset();

        double deltaTime = currentTime - calledTime;

        telemetry.addData("delta time: ", deltaTime);
        double maxChange = MAX_SLEW_RATE * deltaTime;
        double speedChange = targetSpeed - previousSpeed;

        if (Math.abs(speedChange) > maxChange) {
            speedChange = Math.signum(speedChange) * maxChange;
        }

        return previousSpeed + speedChange;
    }

    public void movement(Gamepad gamepad) {
        //calculateMultipliers(gamepad);


        // Apply slew rate limiter to each multiplier
        double limitedLateralSpeed = applySlewRate(-gamepad1.left_stick_x * LATERAL_MULTIPLIER, previousLateralSpeed, lastTime);
        double limitedAxialSpeed = applySlewRate(-gamepad1.left_stick_y * AXIAL_MULTIPLIER, previousAxialSpeed, lastTime);
        double limitedYawSpeed = applySlewRate(-gamepad1.right_stick_x * YAW_MULTIPLIER, previousYawSpeed, lastTime);

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
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        movement(gamepad1);
        currentTime = timer.seconds();
        telemetry.addData("time: ", currentTime);
        telemetry.update();
    }

}

