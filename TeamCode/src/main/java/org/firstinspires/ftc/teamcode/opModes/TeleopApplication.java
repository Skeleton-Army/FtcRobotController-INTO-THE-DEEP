package org.firstinspires.ftc.teamcode.opModes;

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
        telemetry.addData("intake", intakeMotor.getCurrentPosition());
        telemetry.addData("outtake", outtakeMotor.getCurrentPosition());

        movementUtils.fieldCentricMovement();

        // Intake
        if (Debounce.isButtonPressed("a", gamepad2.a)) {
            if (intakeState == ExtensionState.RETRACTED) {
                intakeState = ExtensionState.EXTENDED;
                runAction(intake.extend());
            } else {
                intakeState = ExtensionState.RETRACTED;
                runAction(intake.retract());
            }
        }

        // Outtake
        if (Debounce.isButtonPressed("y", gamepad2.y)) {
            if (outtakeState == ExtensionState.RETRACTED) {
                outtakeState = ExtensionState.EXTENDED;
                runAction(outtake.extend());
            } else {
                outtakeState = ExtensionState.RETRACTED;
                runAction(outtake.retract());
            }
        }

        // Claw
        if (Debounce.isButtonPressed("right_bumper", gamepad2.right_bumper)) {
            runAction(intake.closeClaw());
        } else if (Debounce.isButtonPressed("left_bumper", gamepad2.left_bumper)) {
            runAction(intake.openClaw());
        }

        // Wrist
        if (Debounce.isButtonPressed("right_trigger", gamepad2.right_trigger > 0.1)) {
            runAction(intake.extendWrist());
        } else if (Debounce.isButtonPressed("right_trigger", gamepad2.left_trigger > 0.1)) {
            runAction(intake.retractWrist());
        }

        // Bucket
        if (Debounce.isButtonPressed("x", gamepad2.x)) {
            runAction(outtake.dunk());
        } else if (Debounce.isButtonPressed("b", gamepad2.b)) {
            runAction(outtake.hold());
        }

        // Run all queued actions
        runAllActions();

        // Debugging
        telemetry.update();
    }
}