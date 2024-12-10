package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

@TeleOp(name = "Intake Test", group = "SA_FTC")
public class IntakeTest extends LinearOpMode {
    final double SLOW_MODE_MULTIPLIER = 0.3;

    double axialMultiplier;
    double lateralMultiplier;
    double yawMultiplier;

    DcMotor extend;
    Servo claw;
    Servo wrist;
//    CRServo crServo;

    MecanumDrive drive;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize SampleMecanumDrive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        extend = hardwareMap.get(DcMotor.class, "extend");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
//        crServo = hardwareMap.get(CRServo.class, "testCRServo");

        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            extend.setPower(-gamepad2.right_stick_y);

            if (gamepad2.a) {
                claw.setPosition(IntakeConfig.clawOpen);
            } else if (gamepad2.b) {
                claw.setPosition(IntakeConfig.clawClosed);
            }

            if (gamepad2.x) {
                wrist.setPosition(IntakeConfig.wristRetract);
            } else if (gamepad2.y) {
                wrist.setPosition(IntakeConfig.wristExtend);
            }

//            if (gamepad2.x) {
//                crServo.setPower(1);
//            } else if (gamepad2.y) {
//                crServo.setPower(-1);
//            } else {
//                crServo.setPower(0);
//            }

            // Debugging
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("pos", -extend.getCurrentPosition());
            telemetry.addData("stick y", -gamepad2.right_stick_y);

            telemetry.update();
        }
    }
}