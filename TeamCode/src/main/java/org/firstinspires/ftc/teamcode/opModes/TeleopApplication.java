package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Webcam;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;
import org.firstinspires.ftc.teamcode.utils.general.Debounce;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;
import org.openftc.easyopencv.OpenCvWebcam;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@TeleOp(name = "Teleop App", group = "SA_FTC")
public class TeleopApplication extends TeleopOpMode {
    public static TeleopApplication Instance;

    enum ExtensionState {
        EXTENDED,
        RETRACTED
    }

    public MecanumDrive drive;

    Intake intake;
    Outtake outtake;
    SpecimenArm specimenArm;

    MovementUtils movementUtils;

    ExtensionState intakeState = ExtensionState.RETRACTED;
    ExtensionState outtakeState = ExtensionState.RETRACTED;

    CachingDcMotorEx outtakeMotor;
    CachingDcMotorEx intakeMotor;
    CachingDcMotorEx specimenArmMotor;

    boolean manuallyMoved = false;


    Webcam webcamSequences;
    Drive driveActions;
    Apriltag apriltag;
    OpenCvWebcam webcamOpencv;
    DetectSamples detectSamples;

    WebcamCV webcamCV;
    boolean streaming = false;
    @Override
    public void init() {
        Instance = this;

        telemetry.setMsTransmissionInterval(100); // Default is 250ms

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);

        movementUtils = new MovementUtils(hardwareMap);

        outtakeMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, OuttakeConfig.motorName));
        intakeMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, IntakeConfig.motorName));
        specimenArmMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, SpecimenArmConfig.motorName));

        apriltag = new Apriltag(hardwareMap, drive);
        driveActions = new Drive(drive, apriltag);

        webcamCV = new WebcamCV(hardwareMap, telemetry, drive);

        webcamCV.configureWebcam(SampleColor.YELLOW);
        webcamCV.webcam.stopStreaming(); // let's see if that breaks it
        // webcamOpencv.startStreaming(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2);

        webcamSequences = new Webcam(driveActions, intake, outtake, "red");
    }

    @Override
    public void start() {
        // Enable auto bulk reads
        Utilities.setBulkReadsMode(hardwareMap, LynxModule.BulkCachingMode.AUTO);
    }

    @Override
    public void loop() {
//        movementUtils.fieldCentricMovement();
        movementUtils.movement();

        // Intake
        if (Debounce.isButtonPressed("a", gamepad2.a)) {
            if (intakeState == ExtensionState.RETRACTED) {
                intakeState = ExtensionState.EXTENDED;
                runAction(
                        new ParallelAction(
                                intake.extend(),
                                intake.extendWrist(),
                                intake.openClaw()
                        )
                );
            } else {
                intakeState = ExtensionState.RETRACTED;
                runAction(
                        new ParallelAction(
                                intake.retractWrist(),
                                outtake.hold(),
                                new SequentialAction(
                                        intake.retract(),
                                        intake.openClaw(),
                                        new SleepAction(0.5),
                                        intake.wristMiddle()
                                )
                        )
                );
            }
        }

        if (Debounce.isButtonPressed("right_trigger", gamepad2.right_trigger > 0.1)) {
            runAction(intake.extendWrist());
        } else if (Debounce.isButtonPressed("left_trigger", gamepad2.left_trigger > 0.1)) {
            runAction(intake.retractWrist());
        }

        // Intake Joystick Control
        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            manuallyMoved = true;
            intake.setPower(gamepad2.left_stick_y);
        } else if (manuallyMoved) {
            manuallyMoved = false;
            intake.setPower(0);
        }

        // Outtake
        if (Debounce.isButtonPressed("y", gamepad2.y)) {
            if (outtakeState == ExtensionState.RETRACTED) {
                outtakeState = ExtensionState.EXTENDED;
                runAction(outtake.extend());
            } else {
                outtakeState = ExtensionState.RETRACTED;
                runAction(
                        new SequentialAction(
                                outtake.dunk(),
                                new SleepAction(1),
                                new ParallelAction(
                                        outtake.retract(),
                                        outtake.hold()
                                )
                        )
                );
            }
        }

        // Claw
        if (Debounce.isButtonPressed("right_bumper", gamepad2.right_bumper)) {
            runAction(intake.closeClaw());
        } else if (Debounce.isButtonPressed("left_bumper", gamepad2.left_bumper)) {
            runAction(intake.openClaw());
        }

        // Specimen Arm
        if (Debounce.isButtonPressed("dpad_up", gamepad2.dpad_up)) {
            runAction(specimenArm.armToOuttake());
        } else if (Debounce.isButtonPressed("dpad_down", gamepad2.dpad_down)) {
            runAction(specimenArm.armToIntake());
        }

        // Specimen Grip
        if (Debounce.isButtonPressed("dpad_left", gamepad2.dpad_left)) {
            runAction(specimenArm.gripToOuttake());
        } else if (Debounce.isButtonPressed("dpad_right", gamepad2.dpad_right)) {
            runAction(specimenArm.gripToIntake());
        }

        // cycles actions
        if (Debounce.isButtonPressed("left_bumper", gamepad1.left_bumper)) {
            runAction(driveActions.moveApriltag(new Pose2d(0,0,0)));
        }
        if (Debounce.isButtonPressed("dpad_up", gamepad1.dpad_up)) {
            runAction(webcamSequences.basketCycle());
        }
        if (Debounce.isButtonPressed("dpad_right", gamepad1.dpad_right)) {
            runAction(webcamSequences.specimenCycle());
        }
        if (Debounce.isButtonPressed("dpad_left", gamepad1.dpad_left)) {
            if (streaming) {
                runAction(webcamSequences.pickupSample(webcamCV.getBestSamplePos(new Vector2d(drive.pose.position.x, drive.pose.position.y))));
                webcamOpencv.stopStreaming();
                streaming = false;
            }
        }

        // starts camera streaming for sample detection
        if (Debounce.isButtonPressed("dpad_down", gamepad1.dpad_down)) {
            streaming = true;
            webcamOpencv.startStreaming(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2);
        }


        // Run all queued actions
        runAllActions();

        // Debugging
        telemetry.addData("Intake Position", intakeMotor.getCurrentPosition());
        telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
        telemetry.addData("Outtake Position", outtakeMotor.getCurrentPosition());
        telemetry.addData("Outtake Velocity", outtakeMotor.getVelocity());
        telemetry.addData("Specimen Arm Position", specimenArmMotor.getCurrentPosition());

        telemetry.update();
    }
}