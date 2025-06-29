package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Hang;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.actions.ConditionAction;
import org.firstinspires.ftc.teamcode.utils.actions.ImmediateAction;
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.TeleopControls;
import org.firstinspires.ftc.teamcode.utils.general.MarrowGamepad;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;

import java.util.List;

@TeleOp(name = "Teleop App", group = "SA_FTC")
public class TeleopApplication extends TeleopOpMode {
    enum OuttakeMode {
        HIGH,
        FILLED,
        LOW
    }

    public static TeleopApplication Instance;

    public MecanumDrive drive;

    MarrowGamepad gamepad1;
    MarrowGamepad gamepad2;

    Intake intake;
    Outtake outtake;
    SpecimenArm specimenArm;
    Hang hang;
//    IntakeSensor intakeSensor;

    MovementUtils movementUtils;

    DigitalChannel outtakeSwitch;

    boolean manuallyMoved = false;
    boolean hangExtended = false;

    OuttakeMode outtakeMode = OuttakeMode.HIGH;

    boolean resetOuttake = false;

    TeleopControls controls;

    @Override
    public void init() {
        Instance = this;

        gamepad1 = new MarrowGamepad(this, super.gamepad1);
        gamepad2 = new MarrowGamepad(this, super.gamepad2);

        controls = new TeleopControls(gamepad1, gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);
        hang = new Hang(hardwareMap);
//        intakeSensor = new IntakeSensor(hardwareMap);

        movementUtils = new MovementUtils(super.gamepad1, controls.SLOW_MODE_BUTTON);

        outtakeSwitch = hardwareMap.get(DigitalChannel.class, OuttakeConfig.limitSwitchName);
    }

    @Override
    public void start() {
        // Enable auto bulk reads
        Utilities.setBulkReadsMode(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        runAction(intake.wristMiddle());
        runAction(intake.rotate(0));

        gamepad1.gamepad().rumble(100);
        gamepad2.gamepad().rumble(100);
    }

    @Override
    public void loop() {
//        movementUtils.fieldCentricMovement();
        movementUtils.movement();

        // Run systems
        runIntake();
        runIntakeRotation();
        runManualIntake();
        runGrab();
        runOuttake();
        runSpecimenArm();
        runHang();
        runEmergencyStop();
        runResetMotors();
        outtakeLimitSwitch();

        // Run all queued actions
        runAllActions();

        // Bulk reads from walmart
//        intakeSensor.updateRGBCache();

        // Debugging
        telemetry.addData("Intake Position", intake.motor.getCurrentPosition());
        telemetry.addData("Intake Velocity", intake.motor.getVelocity());
        telemetry.addData("Outtake Position", outtake.motor.getCurrentPosition());
        telemetry.addData("Outtake Velocity", outtake.motor.getVelocity());
        telemetry.addData("Specimen Arm Position", specimenArm.motor.getCurrentPosition());
        telemetry.addData("Hang Position", hang.motor.getCurrentPosition());
        telemetry.addData("Outtake Limit Switch", !outtakeSwitch.getState());
//        telemetry.addData("Intake Color Sensor RGB", intakeSensor.getRGBValues()[0] + "," + intakeSensor.getRGBValues()[1] + "," + intakeSensor.getRGBValues()[2]);
//        telemetry.addData("Got Sample", intakeSensor.gotYellowSample() + " " + intakeSensor.gotRedSample() + " " + intakeSensor.gotBlueSample() + " " + intakeSensor.gotSample());
        telemetry.addData("Intake Rotation Vector", "(" + controls.INTAKE_ROTATION_X.value() + "," + (-controls.INTAKE_ROTATION_Y.value()) + ")");
        telemetry.addData("Intake Rotation", Math.toDegrees(Math.atan2(controls.INTAKE_ROTATION_X.value(), -controls.INTAKE_ROTATION_Y.value())));
        telemetry.addData("Intake Current", intake.motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Outtake Current", outtake.motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Specimen Arm Current", specimenArm.motor.getCurrent(CurrentUnit.AMPS));

        telemetry.update();
    }

    public void runIntake() {
        if (controls.INTAKE_BUTTON.isJustPressed()) {
            runSequentialActions(
                    "intake",

                    // Extend intake
                    new SequentialAction(
                            intake.wristReady(),
                            intake.extend()
                    ),

                    // Retract intake
                    new SequentialAction(
                            intake.closeClaw(),
                            intake.wristMiddle(),
                            intake.rotate(0),
                            new SleepAction(0.2),
                            new ImmediateAction(intake.retract()),

                            new SleepUntilAction(() -> intake.motor.getCurrentPosition() > IntakeConfig.extendPosition * 0.3),

                            new ConditionAction(
                                    new SequentialAction(
                                            intake.retractWrist(),
                                            new SleepAction(0.3),
                                            outtake.hold(),
                                            intake.openClaw(),
                                            intake.wristMiddle(),
                                            new SleepAction(0.2),
                                            outtake.bucketMiddle()
                                    ),
                                    controls.INTAKE_THROW_BUTTON::isUp
                            )
                    )
            );
        }
        else if (controls.INTAKE_BUTTON.isJustReleased() && isInState("intake", 0) && !isActionRunning("intake", 1)) {
            runAction(
                    new SequentialAction(
                            intake.wristMiddle(),
                            new ImmediateAction(intake.extend()),
                            new SleepUntilAction(() -> intake.motor.getCurrentPosition() < IntakeConfig.extendPosition * 0.5),
                            intake.wristReady(),
                            intake.openClaw(),
                            new SleepAction(0.2),
                            intake.wristMiddle(),
                            intake.retract()
                    )
            );
        }
    }

    public void runIntakeRotation() {
        // Intake claw rotation
        if (isInState("intake", 1)) {
            double x = controls.INTAKE_ROTATION_X.value();
            double y = -controls.INTAKE_ROTATION_Y.value();
            double rotation = Math.toDegrees(Math.atan2(x, y)) / 90;

            if (rotation >= -1 && rotation <= 1) {
                runAction(intake.rotate(rotation));
            }
        }
    }

    public void runManualIntake() {
        // Intake manual movement
        if (Math.abs(controls.INTAKE_MANUAL_Y.value()) > 0.1 && isInState("intake", 1) && (controls.INTAKE_MANUAL_Y.value() > 0 || intake.motor.getCurrentPosition() > IntakeConfig.extendPosition)) {
            manuallyMoved = true;
            intake.setPower(controls.INTAKE_MANUAL_Y.value() * IntakeConfig.manualSpeed);
        } else if (manuallyMoved) {
            manuallyMoved = false;
            intake.setPower(0);
        }
    }

    public void runOuttake() {
        if (controls.OUTTAKE_BUTTON.isJustPressed()) {
            runSequentialActions(
                    // Extend outtake
                    new SequentialAction(
                            new SleepUntilAction(() -> !isActionRunning("intake", 1)),
                            new ImmediateAction(outtake.extend(outtakeMode != OuttakeMode.LOW)),
                            new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -400),
                            outtake.bucketReady()
                    ),

                    // Dunk bucket
                    outtakeMode == OuttakeMode.FILLED ? outtake.filledDunk() : outtake.dunk(),

                    // Retract outtake
                    new SequentialAction(
                            outtake.hold(),
                            new SleepUntilAction(() -> !isActionRunning("intake", 1)),
                            new ImmediateAction(outtake.retract())
                    )
            );
        }

        if (controls.OUTTAKE_CYCLE_BUTTON.isJustPressed()) {
            outtakeMode = OuttakeMode.values()[(outtakeMode.ordinal() + 1) % OuttakeMode.values().length]; // Cycle to next mode

            gamepad2.gamepad().rumbleBlips(outtakeMode.ordinal() + 1);
        }
    }

    public void runGrab() {
        if (controls.GRAB_BUTTON.isJustPressed() && isInState("intake", 1)) {
            runAction(
                    new SequentialAction(
                            intake.openClaw(),
                            intake.extendWrist(),
                            new SleepAction(0.25),
                            intake.closeClaw(),
                            new SleepAction(0.2),
                            intake.wristReady()
                    )
            );
        } else if (controls.RELEASE_BUTTON.isJustPressed()) {
            runAction(
                    new SequentialAction(
                            isInState("intake", 1) ? intake.wristReady() : new NullAction(), // Move wrist if intaking samples, else only open claw
                            intake.openClaw()
                    )
            );
        }
    }

    public void runSpecimenArm() {
        Runnable stopOtherActions = () -> {
            stopAction("specimen_outtake");
            stopAction("specimen_hanged");
            stopAction("specimen_intake");
            stopAction("specimen_grab");
        };

        if (controls.SPECIMEN_HANG_BUTTON.isJustPressed()) {
            stopOtherActions.run();

            runAction(
                    "specimen_outtake",
                    new ParallelAction(
                            specimenArm.gripToOuttake(),
                            specimenArm.goToOuttake()
                    )
            );
        } else if (controls.SPECIMEN_HANG_BUTTON.isJustReleased()) {
            stopOtherActions.run();

            runAction(
                    "specimen_hanged",
                    new SequentialAction(
                            specimenArm.goToHanged(),
                            new SleepAction(0.2),
                            specimenArm.grabOpen()
                    )
            );
        }

        if (controls.SPECIMEN_INTAKE_BUTTON.isJustPressed()) {
            stopOtherActions.run();

            runAction(
                    "specimen_intake",
                    new SequentialAction(
                            specimenArm.gripToIntake(),
                            specimenArm.goToIntake(),
                            specimenArm.grabOpen()
                    )
            );
        } else if (controls.SPECIMEN_INTAKE_BUTTON.isJustReleased()) {
            stopOtherActions.run();

            runAction(
                    "specimen_grab",
                    specimenArm.grabClose()
            );
        }

        // Run manual control if dpad is held down
        if (controls.SPECIMEN_INTAKE_BUTTON.isDown() || controls.SPECIMEN_HANG_BUTTON.isDown()) {
            runAction(
                    specimenArm.runManualControl(controls.SPECIMEN_MANUAL_Y.value())
            );
        }
    }

    public void runHang() {
        if (controls.HANG_BUTTON.isJustPressed()) {
            hangExtended = true;

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
        if (controls.EMERGENCY_STOP_BUTTON.isJustPressed()) {
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
        if (controls.RESET_MOTORS_BUTTON.isJustPressed()) {
            intake.resetMotor();
        }
    }

    public void outtakeLimitSwitch() {
        if (!outtakeSwitch.getState()) {
            if (!resetOuttake) {
                resetOuttake = true;
                outtake.resetMotor();
            }
        }
        else {
            resetOuttake = false;
        }
    }
}