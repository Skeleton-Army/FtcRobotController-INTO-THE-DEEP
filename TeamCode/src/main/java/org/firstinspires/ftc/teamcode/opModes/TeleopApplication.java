package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.SpecimenArm;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig;
import org.firstinspires.ftc.teamcode.utils.general.Debounce;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;

import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.d;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.f;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.i;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.p;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.target;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.ticks_in_degree;

enum ExtensionState {
    EXTENDED,
    RETRACTED
}

@TeleOp(name = "Teleop App", group = "SA_FTC")
public class TeleopApplication extends TeleopOpMode {
    public static TeleopApplication Instance;

    public MecanumDrive drive;

    Intake intake;
    Outtake outtake;
    SpecimenArm specimenArm;

    MovementUtils movementUtils;

    ExtensionState intakeState = ExtensionState.RETRACTED;
    ExtensionState outtakeState = ExtensionState.RETRACTED;

    DcMotorEx outtakeMotor;
    DcMotorEx intakeMotor;
    DcMotorEx specimenArmMotor;

    boolean manuallyMoved = false;

    private PIDController controller;

    int armTarget = 0;
    boolean armMoving = false;

    private final ElapsedTime armTimer = new ElapsedTime();

    @Override
    public void init() {
        Instance = this;

        telemetry.setMsTransmissionInterval(100); // Default is 250ms

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);

        movementUtils = new MovementUtils(hardwareMap);

        outtakeMotor = hardwareMap.get(DcMotorEx.class, OuttakeConfig.motorName);
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConfig.motorName);
        specimenArmMotor = hardwareMap.get(DcMotorEx.class, SpecimenArmConfig.motorName);

        controller = new PIDController(p, i, d);
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
            runAction(intake.wristMiddle());
        }

        // Intake Joystick Control
        if (Math.abs(gamepad2.left_stick_y) > 0.1 && intakeState == ExtensionState.EXTENDED) {
            manuallyMoved = true;
            intake.setPower(gamepad2.left_stick_y * IntakeConfig.manualSpeed);
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
            armTarget = -280;
            runAction(specimenArm.gripToOuttake());
        } else if (Debounce.isButtonPressed("dpad_down", gamepad2.dpad_down)) {
            armTimer.reset();
            armTarget = -100;
            armMoving = true;
            runAction(specimenArm.gripToIntake());
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

        controller.setPID(p, i, d);
        int pos = specimenArmMotor.getCurrentPosition();
        double pid = controller.calculate(pos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;

        double power = pid + ff;
        double limitPower = (Math.abs((Math.cos(Math.toRadians((pos + 50) / 2)) )) * 2 * SpecimenArmConfig.power) + 0.15;
        double actualPower = clamp(power, -limitPower, limitPower);

        specimenArmMotor.setPower(actualPower);

        if (armTimer.seconds() > 1.5 && armMoving) {
            armTarget = -45;
            armMoving = false;
        }
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}