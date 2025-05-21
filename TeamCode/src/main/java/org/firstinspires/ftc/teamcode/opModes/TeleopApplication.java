package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Hang;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.IntakeSensor;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.actions.ConditionAction;
import org.firstinspires.ftc.teamcode.utils.actions.NoSleepAction;
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
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

    ElapsedTime runtime = new ElapsedTime();

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
    }

    @Override
    public void start() {
        // Enable auto bulk reads
        Utilities.setBulkReadsMode(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        runAction(intake.wristMiddle());
        runAction(intake.rotate(0));

        runtime.reset();
    }

    @Override
    public void loop() {
//        movementUtils.fieldCentricMovement();
        movementUtils.movement();

        // Run systems
        runIntake();
        runIntakeControls();
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
        telemetry.addData("Gamepad2 X", gamepad2.left_stick_x);
        telemetry.addData("Gamepad2 Y", -gamepad2.left_stick_y);
        telemetry.addData("Intake Rotation", Math.atan2(gamepad2.left_stick_x, -gamepad2.left_stick_y));

        telemetry.update();
    }

    public void runIntake() {
        if (Utilities.isPressed(gamepad2.a)) {
            runSequentialActions(
                    "intake",

                    // Extend intake
                    new SequentialAction(
                            intake.wristReady(),
                            intake.extend(0.75)
                    ),

                    // Retract intake
                    new SequentialAction(
                            intake.closeClaw(),
                            intake.wristMiddle(),
                            intake.rotate(0),
                            new SleepAction(0.2),
                            new NoSleepAction(intake.retract()),

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
                                    () -> !gamepad2.a
                            )
                    )
            );
        }
        else if (Utilities.isReleased(gamepad2.a) && isInState("intake", 0) && !isActionRunning("intake", 1)) {
            runAction(
                    new SequentialAction(
                            intake.wristMiddle(),
                            new NoSleepAction(intake.extend()),
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

    public void runIntakeControls() {
        // Intake claw rotation
        if (isInState("intake", 1)) {
//            double x = Math.ceil(Math.abs(gamepad2.left_stick_x)) * Math.signum(gamepad2.left_stick_x);
//            double y = Math.ceil(Math.abs(gamepad2.left_stick_y)) * Math.signum(-gamepad2.left_stick_y);

//            String key = (int) x + "," + (int) y;
//
//            switch (key) {
//                case "-1,0":
//                    runAction(intake.rotate(-1)); break;
//                case "-1,1":
//                    runAction(intake.rotate(-0.5)); break;
//                case "0,1":
//                    runAction(intake.rotate(0)); break;
//                case "1,1":
//                    runAction(intake.rotate(0.5)); break;
//                case "1,0":
//                    runAction(intake.rotate(1)); break;
//            }

            double x = gamepad2.left_stick_x;
            double y = -gamepad2.left_stick_y;
            double rotation = Math.atan2(x, y);

            if (rotation >= -1 && rotation <= 1) {
                runAction(intake.rotate(rotation));
            }
        }

        // Intake manual movement
        if (Math.abs(gamepad2.right_stick_y) > 0.1 && isInState("intake", 1) && (gamepad2.right_stick_y > 0 || intake.motor.getCurrentPosition() > IntakeConfig.extendPosition)) {
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
                            outtake.extend(outtakeMode != OuttakeMode.LOW),
                            new SequentialAction(
                                    new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -400),
                                    outtake.bucketReady()
                            )
                    ),

                    // Dunk bucket
                    outtakeMode == OuttakeMode.FILLED ? outtake.filledDunk() : outtake.dunk(),

                    // Retract outtake
                    new ParallelAction(
                            outtake.retract(),
                            outtake.hold()
                    )
            );
        }

        if (Utilities.isPressed(gamepad2.back)) {
            outtakeMode = OuttakeMode.values()[(outtakeMode.ordinal() + 1) % OuttakeMode.values().length]; // Cycle to next mode

            gamepad2.rumbleBlips(outtakeMode.ordinal() + 1);
        }
    }

    public void runGrab() {
        if (Utilities.isPressed(gamepad2.right_bumper) && isInState("intake", 1)) {
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
        } else if (Utilities.isPressed(gamepad2.left_bumper)) {
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
                    new SequentialAction(
                            specimenArm.goToHanged(),
                            new SleepAction(0.2),
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

        if (runtime.seconds() >= 90 && !hangExtended) {
            hangExtended = true;

            runAction(
                    hang.middleHang()
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