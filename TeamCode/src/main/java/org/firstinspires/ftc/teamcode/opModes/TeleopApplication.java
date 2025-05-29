package org.firstinspires.ftc.teamcode.opModes;

import androidx.arch.core.executor.DefaultTaskExecutor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Hang;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.debugging.Datalogger;
import org.firstinspires.ftc.teamcode.utils.debugging.FtpUploading;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;

import java.io.File;
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

    MovementUtils movementUtils;

    DigitalChannel outtakeSwitch;

    boolean manuallyMoved = false;

    boolean highBasket = true;

    VoltageSensor battery;
    public File logFile;
    Datalog datalog;
    IMU imu;

    @Override
    public void init() {
        Instance = this;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);
        hang = new Hang(hardwareMap);
//        intakeSensor = new IntakeSensor(hardwareMap);

        movementUtils = new MovementUtils(hardwareMap);

        outtakeSwitch = hardwareMap.get(DigitalChannel.class, OuttakeConfig.limitSwitchName);

        battery = hardwareMap.voltageSensor.get("Control Hub");
        logFile = Datalogger.setupLogFile(null);
        telemetry.addData("Local log file ", logFile.getAbsolutePath());
        telemetry.update();
        datalog = new Datalog(logFile);
        datalog.opModeStatus.set("INIT");
        datalog.writeLine();
        imu = hardwareMap.get(IMU.class, "imu");

        // IMU initialization
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void start() {
        // Enable auto bulk reads
        Utilities.setBulkReadsMode(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        runAction(intake.wristMiddle());
        runAction(intake.rotate(0));
        datalog.opModeStatus.set("RUNNING");
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
        outtakeLimitSwitch();

        // Run all queued actions
        runAllActions();

        // Bulk reads from walmart
//        intakeSensor.updateRGBCache();

        // Debugging to Driver Hub
        telemetry.addData("Intake Position", intake.motor.getCurrentPosition());
        telemetry.addData("Intake Velocity", intake.motor.getVelocity());
        telemetry.addData("Outtake Position", outtake.motor.getCurrentPosition());
        telemetry.addData("Outtake Velocity", outtake.motor.getVelocity());
        telemetry.addData("Specimen Arm Position", specimenArm.motor.getCurrentPosition());
        telemetry.addData("Hang Position", hang.motor.getCurrentPosition());
        telemetry.addData("Outtake Limit Switch", !outtakeSwitch.getState());
        telemetry.addData("Gamepad2 X", gamepad2.left_stick_x);
        telemetry.addData("Gamepad2 Y", -gamepad2.left_stick_y);
        telemetry.addData("Current voltage: " , battery.getVoltage());
        telemetry.addData("Yaw", datalog.yaw);
        telemetry.addData("Pitch", datalog.pitch);
        telemetry.addData("Roll", datalog.roll);
        telemetry.addData("OpMode Status", datalog.opModeStatus);
        telemetry.addData("Loop Counter", datalog.loopCounter);
        telemetry.addData("Battery", datalog.battery);
        for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
            telemetry.addData("Voltage Sensor Name", sensor.getDeviceName());
            telemetry.addData("Voltage", sensor.getVoltage());
        }

        telemetry.update();
        // Debugging to log file
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        datalog.loopCounter.set(time);
        datalog.battery.set(battery.getVoltage());
        datalog.intakePos.set(intake.motor.getCurrentPosition());
        datalog.intakeVel.set(intake.motor.getVelocity());
        datalog.outtakePos.set(outtake.motor.getCurrentPosition());
        datalog.outtakeVel.set(outtake.motor.getVelocity());
        datalog.specArmPos.set(specimenArm.motor.getCurrentPosition());
        datalog.hangPos.set(hang.motor.getCurrentPosition());
        datalog.outtakeLimit.set(!outtakeSwitch.getState());
        datalog.gamepad2X.set(gamepad2.left_stick_x);
        datalog.gamepad2Y.set(-gamepad2.left_stick_y);
        datalog.robotAngle.set(Math.toDegrees(drive.pose.heading.toDouble()));
        datalog.robotPosX.set(drive.pose.position.x);
        datalog.robotPosY.set(drive.pose.position.y);
        datalog.yaw.set(orientation.getYaw());
        datalog.pitch.set(orientation.getPitch());
        datalog.roll.set(orientation.getRoll());
        datalog.writeLine();
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
    public void stop() {
        try {
            FtpUploading ftpUploading = new FtpUploading();
            ftpUploading.UploadFile(logFile, "/" + logFile.getName(), FtpUploading.ASCII, true);
            ftpUploading.disconnect();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
    public static class Datalog {
        private final Datalogger datalogger;
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField intakePos = new Datalogger.GenericField("Intake Position");
        public Datalogger.GenericField intakeVel = new Datalogger.GenericField("Intake Velocity");
        public Datalogger.GenericField outtakePos = new Datalogger.GenericField("Outtake Position");
        public Datalogger.GenericField outtakeVel = new Datalogger.GenericField("Outtake Velocity");
        public Datalogger.GenericField specArmPos = new Datalogger.GenericField("SpecimenArm Position");
        public Datalogger.GenericField hangPos = new Datalogger.GenericField("Hang Position");
        public Datalogger.GenericField outtakeLimit = new Datalogger.GenericField("Outtake Limit Switch");
        public Datalogger.GenericField gamepad2X = new Datalogger.GenericField("Gamepad 2 X");
        public Datalogger.GenericField gamepad2Y = new Datalogger.GenericField("Gamepad 2 Y");
        public Datalogger .GenericField battery = new Datalogger.GenericField("Battery");
        public Datalogger.GenericField robotPosX = new Datalogger.GenericField("Robot Position X");
        public Datalogger.GenericField robotPosY = new Datalogger.GenericField("Robot Position Y");
        public Datalogger.GenericField robotAngle = new Datalogger.GenericField("Robot Angle");
        public Datalogger.GenericField yaw          = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField pitch        = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField roll         = new Datalogger.GenericField("Roll");
        public Datalog(File logFile) {
            datalogger = Datalogger.builder()
                    .setFilename(logFile)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    .setFields(
                            opModeStatus,
                            loopCounter,
                            intakePos,
                            intakeVel,
                            outtakePos,
                            outtakeVel,
                            specArmPos,
                            hangPos,
                            outtakeLimit,
                            gamepad2X,
                            gamepad2Y,
                            battery,
                            robotAngle,
                            robotPosX,
                            robotPosY,
                            yaw,
                            pitch,
                            roll
                    )
                    .build();
        }
        public void writeLine() {
            datalogger.writeLine();
        }
    }
}