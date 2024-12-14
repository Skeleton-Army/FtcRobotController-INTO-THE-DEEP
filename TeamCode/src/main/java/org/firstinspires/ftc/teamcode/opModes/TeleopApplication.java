package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;
import org.firstinspires.ftc.teamcode.utils.general.Debounce;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;

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

    MovementUtils movementUtils;

    ExtensionState intakeState = ExtensionState.RETRACTED;
    ExtensionState outtakeState = ExtensionState.RETRACTED;

    DcMotorEx outtakeMotor;
    DcMotorEx intakeMotor;

    @Override
    public void init() {
        Instance = this;

        telemetry.setMsTransmissionInterval(100); // Default is 250ms

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        movementUtils = new MovementUtils(hardwareMap);

        outtakeMotor = hardwareMap.get(DcMotorEx.class, OuttakeConfig.motorName);
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConfig.motorName);
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
                                        intake.wristMiddle()
                                )
                        )
                );
            }
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

        // Run all queued actions
        runAllActions();

        // Debugging
        telemetry.addData("Intake Position", intakeMotor.getCurrentPosition());
        telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
        telemetry.addData("Outtake Position", outtakeMotor.getCurrentPosition());
        telemetry.addData("Outtake Velocity", outtakeMotor.getVelocity());

        telemetry.update();
    }
}