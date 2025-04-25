package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.skeletonarmy.marrow.actions.SleepUntilAction;
import com.skeletonarmy.marrow.MarrowGamepad;
import com.skeletonarmy.marrow.TeleopOpMode;
import com.skeletonarmy.marrow.MarrowUtils;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Hang;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;

import java.util.List;

@TeleOp(name = "Teleop App", group = "SA_FTC")
public class TeleopApplication extends TeleopOpMode {
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

    boolean highBasket = true;

    boolean resetOuttake = false;

    @Override
    public void init() {
        Instance = this;

        gamepad1 = new MarrowGamepad(super.gamepad1);
        gamepad2 = new MarrowGamepad(super.gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);
        hang = new Hang(hardwareMap);
//        intakeSensor = new IntakeSensor(hardwareMap);

        movementUtils = new MovementUtils(hardwareMap, super.gamepad1, super.gamepad2);

        outtakeSwitch = hardwareMap.get(DigitalChannel.class, OuttakeConfig.limitSwitchName);
    }

    @Override
    public void start() {
        // Enable auto bulk reads
        MarrowUtils.setBulkReadsMode(hardwareMap, LynxModule.BulkCachingMode.AUTO);

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
        telemetry.addData("Gamepad2 X", gamepad2.left_stick_x.value());
        telemetry.addData("Gamepad2 Y", -gamepad2.left_stick_y.value());

        telemetry.update();
    }

    public void runIntakeWithDeposit() {
        if (gamepad2.a.isJustPressed()) {
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
        if (gamepad2.x.isJustPressed()) {
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
            double x = Math.ceil(Math.abs(gamepad2.left_stick_x.value())) * Math.signum(gamepad2.left_stick_x.value());
            double y = Math.ceil(Math.abs(gamepad2.left_stick_y.value())) * Math.signum(-gamepad2.left_stick_y.value());

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
        if (Math.abs(gamepad2.right_stick_y.value()) > 0.1 && isInState("intake", 1) && (gamepad2.right_stick_y.value() > 0 || intake.motor.getCurrentPosition() < IntakeConfig.extendPosition)) {
            manuallyMoved = true;
            intake.setPower(gamepad2.right_stick_y.value() * IntakeConfig.manualSpeed);
        } else if (manuallyMoved) {
            manuallyMoved = false;
            intake.setPower(0);
        }
    }

    public void runOuttake() {
        if (gamepad2.y.isJustPressed() && !isActionRunning("intake", 1)) {
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

        if (gamepad2.back.isJustPressed()) {
            highBasket = !highBasket;
            gamepad2.gamepad().rumble(200);
        }
    }

    public void runWrist() {
        if (gamepad2.right_trigger.isJustPressed()) {
            runAction(intake.extendWrist());
        } else if (gamepad2.left_trigger.isJustPressed()) {
            runAction(
                    new ParallelAction(
                            intake.wristReady(),
                            intake.openClaw()
                    )
            );
        }
    }

    public void runClaw() {
        if (gamepad2.right_bumper.isJustPressed()) {
            runAction(intake.closeClaw());
        } else if (gamepad2.left_bumper.isJustPressed()) {
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

        if (gamepad2.dpad_up.isJustPressed()) {
            stopOtherActions.run();

            runAction(
                    "specimen_outtake",
                    new ParallelAction(
                            specimenArm.gripToOuttake(),
                            specimenArm.goToOuttake()
                    )
            );
        } else if (gamepad2.dpad_up.isJustReleased()) {
            stopOtherActions.run();

            runAction(
                    "specimen_hanged",
                    new ParallelAction(
                            specimenArm.goToHanged(),
                            specimenArm.grabOpen()
                    )
            );
        }

        if (gamepad2.dpad_down.isJustPressed()) {
            stopOtherActions.run();

            runAction(
                    "specimen_intake",
                    new SequentialAction(
                            specimenArm.gripToIntake(),
                            specimenArm.goToIntake(),
                            specimenArm.grabOpen()
                    )
            );
        } else if (gamepad2.dpad_down.isJustReleased()) {
            stopOtherActions.run();

            runAction(
                    "specimen_grab",
                    specimenArm.grabClose()
            );
        }

        // Run manual control if dpad is held down
        if (gamepad2.dpad_down.isDown() || gamepad2.dpad_up.isDown()) {
            runAction(
                    specimenArm.runManualControl(gamepad2.right_stick_y.value())
            );
        }

        specimenArm.update();
    }

    public void runHang() {
        if (gamepad2.start.isJustPressed()) {
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
        if (gamepad2.b.isJustPressed()) {
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
        if (gamepad2.guide.isJustPressed()) {
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