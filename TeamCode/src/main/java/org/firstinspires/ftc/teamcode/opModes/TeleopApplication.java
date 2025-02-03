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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Hang;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;
import org.firstinspires.ftc.teamcode.utils.general.Debounce;
import org.firstinspires.ftc.teamcode.utils.general.GamepadHelper;
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

    DcMotorEx outtakeMotor;
    DcMotorEx intakeMotor;
    DcMotorEx specimenArmMotor;

    DigitalChannel outtakeSwitch;

    boolean manuallyMoved = false;

    boolean armMoving = false;

    boolean highBasket = true;

    private final ElapsedTime armTimer = new ElapsedTime();

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

        outtakeMotor = hardwareMap.get(DcMotorEx.class, OuttakeConfig.motorName);
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConfig.motorName);
        specimenArmMotor = hardwareMap.get(DcMotorEx.class, SpecimenArmConfig.motorName);
        outtakeSwitch = hardwareMap.get(DigitalChannel.class, OuttakeConfig.limitSwitchName);
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
        telemetry.addData("Intake Position", intakeMotor.getCurrentPosition());
        telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
        telemetry.addData("Outtake Position", outtakeMotor.getCurrentPosition());
        telemetry.addData("Outtake Velocity", outtakeMotor.getVelocity());
        telemetry.addData("Specimen Arm Position", specimenArmMotor.getCurrentPosition());
        telemetry.addData("Outtake Limit Switch", outtakeSwitch.getState());

        telemetry.update();

        // Tune PID
        specimenArm.setPID(SpecimenArmConfig.p, SpecimenArmConfig.i, SpecimenArmConfig.d);
    }

    public void runIntakeWithDeposit() {
        if (GamepadHelper.isButtonPressed(gamepad2.a)) {
            runToggleAction(
                    "extend_intake",
                    new SequentialAction(
                            intake.extend(),
                            intake.extendWrist(),
                            intake.openClaw()
                    ),

                    "retract_intake",
                    new ParallelAction(
                            intake.retractWrist(),
                            outtake.hold(),
                            new SequentialAction(
                                    intake.retract(),
                                    intake.openClaw(),
                                    new SleepAction(0.2),
                                    intake.wristMiddle(),
                                    new SleepAction(0.2)
                            )
                    )
            );
        }
    }

    public void runIntake() {
        if (GamepadHelper.isButtonPressed(gamepad2.x)) {
            runToggleAction(
                    "extend_intake",
                    new SequentialAction(
                            intake.extend(),
                            intake.extendWrist(),
                            intake.openClaw()
                    ),

                    "retract_intake",
                    new ParallelAction(
                            intake.retract(),
                            intake.wristMiddle()
                    )
            );
        }
    }

    public void runOuttake() {
        telemetry.addData("bruhhh1", GamepadHelper.getCallSiteHash());
        telemetry.addData("bruhhh2", GamepadHelper.getCallSiteHash());
        if (GamepadHelper.isButtonPressed(gamepad2.y) && !isActionRunning("retract_intake")) {
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

        if (GamepadHelper.isButtonPressed(gamepad2.back)) {
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
        if (GamepadHelper.isButtonPressed(gamepad2.right_trigger > 0.1)) {
            runAction(intake.extendWrist());
        } else if (GamepadHelper.isButtonPressed(gamepad2.left_trigger > 0.1)) {
            runAction(intake.wristMiddle());
        }
    }

    public void runClaw() {
        if (GamepadHelper.isButtonPressed(gamepad2.right_bumper)) {
            runAction(intake.closeClaw());
        } else if (GamepadHelper.isButtonPressed(gamepad2.left_bumper)) {
            runAction(intake.openClaw());
        }
    }

    public void runSpecimenArm() {
        if (GamepadHelper.isButtonPressed(gamepad2.dpad_up)) {
            //runAction(specimenArm.grabToIntake());
            specimenArm.setTarget(SpecimenArmConfig.outtakePosition);
        } else if (GamepadHelper.isButtonPressed(gamepad2.dpad_down)) {
            armTimer.reset();
            armMoving = true;
            //runAction(specimenArm.grabToOuttake());
            specimenArm.setTarget(SpecimenArmConfig.middlePosition);
            runAction(specimenArm.gripToIntake());
        } else if (GamepadHelper.isButtonPressed(gamepad2.dpad_right)) {
            specimenArm.setTarget(SpecimenArmConfig.disabledPosition);
        }

        if (armTimer.seconds() > 1 && armMoving) {
            armMoving = false;

            specimenArm.setTarget(SpecimenArmConfig.intakePosition);
        }

        if (specimenArmMotor.getCurrentPosition() < -250 && !armMoving) {
            runAction(specimenArm.gripToOuttake());
            //runAction(specimenArm.grabToOuttake());
        }

        if (GamepadHelper.isButtonPressed(gamepad2.dpad_left)) {
            runToggleAction(
                    "open_grip",
                    specimenArm.grabOpen(),

                    "close_grip",
                    specimenArm.grabClose()
            );
        }

        telemetry.addData("specimen power: ", specimenArm.calculateArmPower());
        specimenArm.update();
    }

    public void runHang() {
        if (GamepadHelper.isButtonPressed(gamepad1.guide)) {
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
        if (GamepadHelper.isButtonPressed(gamepad2.b)) {
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
        if (GamepadHelper.isButtonPressed(gamepad2.guide)) {
            intake.resetMotor();
            outtake.resetMotor();
        }
    }

    public void outtakeLimitSwitch() {
        if (GamepadHelper.isButtonPressed(outtakeSwitch.getState())) {

        }
    }
}