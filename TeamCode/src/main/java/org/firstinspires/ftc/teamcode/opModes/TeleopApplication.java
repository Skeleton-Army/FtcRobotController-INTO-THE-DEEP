package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Hang;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Webcam;
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagSamplesPipeline;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;

import java.util.List;

@TeleOp(name = "Teleop App", group = "SA_FTC")
public class TeleopApplication extends TeleopOpMode {
    public static TeleopApplication Instance;

    public MecanumDrive drive;

    Intake intake;
    Outtake outtake;
    SpecimenArm specimenArm;
    Hang hang;
//    IntakeSensor intakeSensor;

    Drive actionsDrive;
    Webcam actionCam;
    MovementUtils movementUtils;

    DigitalChannel outtakeSwitch;

    WebcamCV camCV;

    AprilTagSamplesPipeline aprilTagSamplesPipeline;
    boolean manuallyMoved = false;

    boolean highBasket = true;

    Canvas c;
    //TelemetryPacket telemetryPacket;

    @Override
    public void init() {
        Instance = this;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //
        // telemetryPacket = new TelemetryPacket();

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        camCV = new WebcamCV(hardwareMap, telemetry, drive, true, false);
        camCV = new WebcamCV(hardwareMap, telemetry, drive, true, true);
        camCV.configureWebcam(new SampleColor[] { SampleColor.YELLOW, SampleColor.RED}); // TODO: find a way to select an alliance for correct sequences
        aprilTagSamplesPipeline = camCV.getAprilTagSamplesPipeline();

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);
        hang = new Hang(hardwareMap);
//        intakeSensor = new IntakeSensor(hardwareMap);
        actionsDrive = new Drive(drive, camCV, telemetry, aprilTagSamplesPipeline);
        actionCam = new Webcam(actionsDrive, intake, outtake, "red"); // TODO: find a way to select an alliance for correct sequences

        movementUtils = new MovementUtils(hardwareMap);

        outtakeSwitch = hardwareMap.get(DigitalChannel.class, OuttakeConfig.limitSwitchName);

        c = new Canvas();
    }

    @Override
    public void start() {
        // Enable auto bulk reads
        Utilities.setBulkReadsMode(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        runAction(intake.wristMiddle());
        runAction(intake.rotate(0));
    }

    @Override
    public void loop() {
//        movementUtils.fieldCentricMovement();
        movementUtils.movement();

        // Run systems
        runIntakeWithDeposit();
        runIntake();
        runIntakeControls();
        runWrist();
        runOuttake();
        runClaw();
        runSpecimenArm();
        runHang();
        runEmergencyStop();
        runResetMotors();
        runDriverActions();
        outtakeLimitSwitch();

        // Run all queued actions
        runAllActions();

        // Bulk reads from walmart
//        intakeSensor.updateRGBCache();

        drive.updatePoseEstimate();

        TelemetryPacket telemetryPacket = new TelemetryPacket();
        //c = telemetryPacket.field();
        telemetryPacket.fieldOverlay().setStroke("blue");
        Drawing.drawRobot(telemetryPacket.fieldOverlay(), drive.pose, "blue"); // where the robot thinks he is
       // Drawing.drawRobot(c, aprilTagSamplesPipeline.getRobotPosByAprilTag(), "green"); // where the AprilTag thinks the robot is
        // should be the same for now

        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);

        // Debugging
        //telemetry.addData("Intake Position", intake.motor.getCurrentPosition());
        telemetry.addData("Intake Velocity", intake.motor.getVelocity());
        //telemetry.addData("Outtake Position", outtake.motor.getCurrentPosition());
        telemetry.addData("Outtake Velocity", outtake.motor.getVelocity());
        //telemetry.addData("Specimen Arm Position", specimenArm.motor.getCurrentPosition());
        //telemetry.addData("Hang Position", hang.motor.getCurrentPosition());
        //telemetry.addData("Outtake Limit Switch", !outtakeSwitch.getState());
//        telemetry.addData("Intake Color Sensor RGB", intakeSensor.getRGBValues()[0] + "," + intakeSensor.getRGBValues()[1] + "," + intakeSensor.getRGBValues()[2]);
//        telemetry.addData("Got Sample", intakeSensor.gotYellowSample() + " " + intakeSensor.gotRedSample() + " " + intakeSensor.gotBlueSample() + " " + intakeSensor.gotSample());
        //telemetry.addData("Gamepad2 X", gamepad2.left_stick_x);
        //telemetry.addData("Gamepad2 Y", -gamepad2.left_stick_y);

        telemetry.update();
    }

    public void runDriverActions() {
        telemetry.addData("robot pos by apriltag: ",aprilTagSamplesPipeline.getRobotPosByAprilTag().position);
        telemetry.addData("robot angle by apriltag: ",Math.toRadians(aprilTagSamplesPipeline.getRobotPosByAprilTag().heading.toDouble()));
        telemetry.addData("robot pos: ", drive.pose.position);
        if (aprilTagSamplesPipeline.getApriltagDetection() != null)
            telemetry.addData("robot pos by apriltag: ",aprilTagSamplesPipeline.getApriltagDetection().ftcPose);

        camCV.lookForSamples();
        Sample bestSample = camCV.getBestSample(drive.pose.position);

        // drawing the best sample (the selected sample to pickup)

        if(bestSample != null) {
            telemetry.addData("sample detected x pos: ", bestSample.getSamplePosition().position.x);
            telemetry.addData("sample detected y pos: ", bestSample.getSamplePosition().position.y);
            c.fillRect(bestSample.getSamplePosition().position.x, bestSample.getSamplePosition().position.y, 7,7);
        }
        drive.updatePoseEstimate();

        if (Utilities.isPressed(gamepad1.a) && bestSample != null) { // running the alignToSample sequence
            runAction("driver sequence",actionsDrive.alignToSample(bestSample));
        }
        if (Utilities.isPressed(gamepad1.b)) { // running the pickupSample sequence
            runAction("driver sequence",actionCam.pickupSample(bestSample));
        }
        if (Utilities.isPressed(gamepad1.y)) { // running basketCycle sequence, only could run when an apriltag is in sight
            runAction("driver sequence",actionCam.basketCycle());
        }
        if (Utilities.isPressed(gamepad1.x)) { // running specimenCycle sequence, only could run when an apriltag is in sight
            runAction("driver sequence",actionsDrive.moveApriltag(new Pose2d(0,0,0)));
        }
        if (Utilities.isPressed(gamepad1.guide)) { // super cycle! puts the sample in the basket and go to the submersible to get another one
            runSequentialActions("driver sequence",
                    actionCam.basketCycle(),
                    actionsDrive.goToSubmersible(),
                    actionCam.pickupSample(bestSample)
                    );
        }
        if(Utilities.isPressed(gamepad1.right_bumper)) { // stops the current driver action
            stopAction("driver sequence");
        }
    }

    public void runIntakeWithDeposit() {
        if (Utilities.isPressed(gamepad2.a)) {
            runSequentialActions(
                    "intake",

                    // Extend intake
                    new SequentialAction(
                            intake.wristReady(),
                            intake.extend(),
                            intake.openClaw()
                    ),

                    // Retract intake
                    new ParallelAction(
                            intake.closeClaw(),
                            intake.retractWrist(),
                            intake.rotate(0),
                            outtake.hold(),
                            new SequentialAction(
                                    new SleepAction(0.2),
                                    intake.retract(),
                                    intake.openClaw(),
                                    intake.wristMiddle(),
                                    new SleepAction(0.2),
                                    outtake.bucketMiddle()
                            )
                    )
            );
        }
    }

    public void runIntake() {
        if (Utilities.isPressed(gamepad2.x)) {
            runSequentialActions(
                    "intake",

                    // Extend intake
                    new ParallelAction(
                            intake.extend(),
                            intake.wristReady()
                    ),

                    // Retract intake
                    new SequentialAction(
                            intake.wristMiddle(),
                            new SleepAction(0.3),
                            intake.rotate(0),
                            intake.retract()
                    )
            );
        }
    }

    public void runIntakeControls() {
        // Intake claw rotation
        if (isInState("intake", 1)) {
            double x = Math.ceil(Math.abs(gamepad2.left_stick_x)) * Math.signum(gamepad2.left_stick_x);
            double y = Math.ceil(Math.abs(gamepad2.left_stick_y)) * Math.signum(-gamepad2.left_stick_y);

            String key = (int) x + "," + (int) y;

            switch (key) {
                case "-1,0": runAction(intake.rotate(-1)); break;
                case "-1,1": runAction(intake.rotate(-0.5)); break;
                case "0,1": runAction(intake.rotate(0)); break;
                case "1,1": runAction(intake.rotate(0.5)); break;
                case "1,0": runAction(intake.rotate(1)); break;
            }
        }

        // Intake manual movement
        if (Math.abs(gamepad2.right_stick_y) > 0.1 && isInState("intake", 1) && (gamepad2.right_stick_y > 0 || intake.motor.getCurrentPosition() < IntakeConfig.extendPosition)) {
            manuallyMoved = true;
            intake.setPower(gamepad2.right_stick_y * IntakeConfig.manualSpeed);
        } else if (manuallyMoved) {
            manuallyMoved = false;
            intake.setPower(0);
        }
    }

    public void runOuttake() {
        if (Utilities.isPressed(gamepad2.y) && !isActionRunning("intake", 1)) {
            runSequentialActions(
                    // Extend outtake
                    new ParallelAction(
                            outtake.extend(highBasket),
                            new SequentialAction(
                                    new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -400),
                                    outtake.bucketReady()
                            )
                    ),

                    // Dunk bucket
                    outtake.dunk(),

                    // Retract outtake
                    new ParallelAction(
                            outtake.retract(),
                            outtake.hold()
                    )
            );
        }

        if (Utilities.isPressed(gamepad2.back)) {
            highBasket = !highBasket;
            gamepad2.rumble(200);
        }
    }

    public void runWrist() {
        if (Utilities.isPressed(gamepad2.right_trigger > 0.1)) {
            runAction(intake.extendWrist());
        } else if (Utilities.isPressed(gamepad2.left_trigger > 0.1)) {
            runAction(
                    new ParallelAction(
                            intake.wristReady(),
                            intake.openClaw()
                    )
            );
        }
    }

    public void runClaw() {
        if (Utilities.isPressed(gamepad2.right_bumper)) {
            runAction(intake.closeClaw());
        } else if (Utilities.isPressed(gamepad2.left_bumper)) {
            runAction(intake.openClaw());
        }
    }

    public void runSpecimenArm() {
        Runnable stopOtherActions = () -> {
            stopAction("specimen_outtake");
            stopAction("specimen_hanged");
            stopAction("specimen_intake");
            stopAction("specimen_grab");
        };

        if (Utilities.isPressed(gamepad2.dpad_up)) {
            stopOtherActions.run();

            runAction(
                    "specimen_outtake",
                    new ParallelAction(
                            specimenArm.gripToOuttake(),
                            specimenArm.goToOuttake()
                    )
            );
        } else if (Utilities.isReleased(gamepad2.dpad_up)) {
            stopOtherActions.run();

            runAction(
                    "specimen_hanged",
                    new ParallelAction(
                            specimenArm.goToHanged(),
                            specimenArm.grabOpen()
                    )
            );
        }

        if (Utilities.isPressed(gamepad2.dpad_down)) {
            stopOtherActions.run();

            runAction(
                    "specimen_intake",
                    new SequentialAction(
                            specimenArm.gripToIntake(),
                            specimenArm.goToIntake(),
                            specimenArm.grabOpen()
                    )
            );
        } else if (Utilities.isReleased(gamepad2.dpad_down)) {
            stopOtherActions.run();

            runAction(
                    "specimen_grab",
                    specimenArm.grabClose()
            );
        }

        // Run manual control if dpad is held down
        if (gamepad2.dpad_down || gamepad2.dpad_up) {
            runAction(
                    specimenArm.runManualControl(gamepad2.right_stick_y)
            );
        }

        specimenArm.update();
    }

    public void runHang() {
        if (Utilities.isPressed(gamepad2.start)) {
            runSequentialActions(
                    // Ready hang
                    hang.middleHang(),

                    // Extend hang
                    hang.extendHang(),

                    // Retract hang
                    hang.retractHang()
            );
        }
    }

    public void runEmergencyStop() {
        if (Utilities.isPressed(gamepad2.b)) {
            stopAllActions();

            // Stop all motors
            List<DcMotorEx> allMotors = hardwareMap.getAll(DcMotorEx.class);

            for (DcMotorEx motor : allMotors) {
                if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    motor.setTargetPosition(motor.getCurrentPosition());
                }
            }
        }
    }

    public void runResetMotors() {
        if (Utilities.isPressed(gamepad2.guide)) {
            intake.resetMotor();
        }
    }

    public void outtakeLimitSwitch() {
        if (Utilities.isPressed(!outtakeSwitch.getState())) {
            outtake.resetMotor();
        }
    }
}