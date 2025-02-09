package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Hang;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
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

    MovementUtils movementUtils;

    DigitalChannel outtakeSwitch;

    boolean manuallyMoved = false;

    boolean highBasket = true;

    @Override
    public void init() {
        Instance = this;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);
        hang = new Hang(hardwareMap);

        movementUtils = new MovementUtils(hardwareMap);

        outtakeSwitch = hardwareMap.get(DigitalChannel.class, OuttakeConfig.limitSwitchName);
        outtakeSwitch.setMode(DigitalChannel.Mode.INPUT);
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

        // Run systems
        runIntakeWithDeposit();
        runIntake();
        runWrist();
        runManualIntakeControl();
        runOuttake();
        runClaw();
        runSpecimenArm();
        runHang();
        runEmergencyStop();
        runResetMotors();
        outtakeLimitSwitch();

        // Run all queued actions
        runAllActions();

        // Debugging
        telemetry.addData("Intake Position", intake.motor.getCurrentPosition());
        telemetry.addData("Intake Velocity", intake.motor.getVelocity());
        telemetry.addData("Outtake Position", outtake.motor.getCurrentPosition());
        telemetry.addData("Outtake Velocity", outtake.motor.getVelocity());
        telemetry.addData("Specimen Arm Position", specimenArm.motor.getCurrentPosition());
        telemetry.addData("Outtake Limit Switch", !outtakeSwitch.getState());

        telemetry.update();
    }

    public void runIntakeWithDeposit() {
        if (Utilities.isPressed(gamepad2.a)) {
            runToggleAction(
                    "extend_intake",
                    new SequentialAction(
                            intake.extend(),
                            intake.openClaw()
                    ),

                    "retract_intake",
                    new ParallelAction(
                            intake.retractWrist(),
                            outtake.hold(),
                            new SequentialAction(
                                    new ParallelAction(
                                            intake.retract(),
                                            new SleepAction(0.8)
                                    ),
                                    intake.openClaw(),
                                    intake.wristMiddle(),
                                    new SleepAction(0.2)
                            )
                    )
            );
        }
    }

    public void runIntake() {
        if (Utilities.isPressed(gamepad2.x)) {
            runToggleAction(
                    "extend_intake",
                    intake.extend(),

                    "retract_intake",
                    new ParallelAction(
                            intake.retract(),
                            intake.wristMiddle()
                    )
            );
        }
    }

    public void runOuttake() {
        if (Utilities.isPressed(gamepad2.y) && !isActionRunning("retract_intake")) {
            runToggleAction(
                    "extend_outtake",
                    new SequentialAction(
                            outtake.extend(highBasket),
                            outtake.dunk()
                    ),

                    "retract_outtake",
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

    public void runManualIntakeControl() {
        if (Math.abs(gamepad2.left_stick_y) > 0.1 && isInState("extend_intake")) {
            manuallyMoved = true;
            intake.setPower(gamepad2.left_stick_y * IntakeConfig.manualSpeed);
        } else if (manuallyMoved) {
            manuallyMoved = false;
            intake.setPower(0);
        }
    }

    public void runWrist() {
        if (Utilities.isPressed(gamepad2.right_trigger > 0.1)) {
            runAction(intake.extendWrist());
        } else if (Utilities.isPressed(gamepad2.left_trigger > 0.1)) {
            runAction(intake.wristReady());
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
        if (Utilities.isPressed(gamepad2.dpad_up)) {
            specimenArm.setTarget(SpecimenArmConfig.outtakePosition);
            runAction(
                    new SequentialAction(
                            new SleepAction(0.2),
                            specimenArm.gripToOuttake()
                    )
            );
        } else if (Utilities.isPressed(gamepad2.dpad_down)) {
            specimenArm.setTarget(SpecimenArmConfig.intakePosition);
            runAction(
                    new ParallelAction(
                            specimenArm.grabOpen(),
                            specimenArm.gripToIntake()
                    )
            );
        }

        if (Utilities.isPressed(gamepad2.dpad_left)) {
            runToggleAction(
                    "open_grip",
                    specimenArm.grabOpen(),

                    "close_grip",
                    specimenArm.grabClose()
            );
        }

        specimenArm.update();
    }

    public void runHang() {
        if (Utilities.isPressed(gamepad1.guide)) {
            runToggleAction(
                    "extend_hang",
                    new ParallelAction(
                            hang.extendHang(),
                            hang.extendOuttake()
                    ),

                    "retract_hang",
                    new ParallelAction(
                            hang.retractHang(),
                            hang.retractOuttake()
                    )
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