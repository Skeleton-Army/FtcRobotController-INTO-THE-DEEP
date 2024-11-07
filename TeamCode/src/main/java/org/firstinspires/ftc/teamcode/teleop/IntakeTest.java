package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "Intake Test", group = "SA_FTC")
public class IntakeTest extends LinearOpMode {
    final double SLOW_MODE_MULTIPLIER = 0.3;

    double axialMultiplier;
    double lateralMultiplier;
    double yawMultiplier;

    DcMotor armExtend;
    Servo servo;
    CRServo crServo;

    MecanumDrive drive;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize SampleMecanumDrive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        servo = hardwareMap.get(Servo.class, "testServo");
        crServo = hardwareMap.get(CRServo.class, "testCRServo");

        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean slowModeActive = gamepad1.right_bumper;

            axialMultiplier = (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
            lateralMultiplier = (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
            yawMultiplier = (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(gamepad1.left_stick_y * axialMultiplier, -gamepad1.left_stick_x * lateralMultiplier),
                            -gamepad1.right_stick_x * yawMultiplier
                    )
            );

            drive.updatePoseEstimate();

//            if ((-armExtend.getCurrentPosition() < 2000 || -gamepad2.right_stick_y < 0) && (-armExtend.getCurrentPosition() > 300 || -gamepad2.right_stick_y > 0)) {
//                armExtend.setPower(-gamepad2.right_stick_y);
//            }

            armExtend.setPower(-gamepad2.right_stick_y);

            if (gamepad2.a) {
                servo.setPosition(0.5);
            } else if (gamepad2.b) {
                servo.setPosition(0);
            }

            if (gamepad2.x) {
                crServo.setPower(1);
            } else if (gamepad2.y) {
                crServo.setPower(-1);
            } else {
                crServo.setPower(0);
            }

            // Debugging
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("pos", -armExtend.getCurrentPosition());
            telemetry.addData("stick y", -gamepad2.right_stick_y);

            telemetry.update();
        }
    }
}