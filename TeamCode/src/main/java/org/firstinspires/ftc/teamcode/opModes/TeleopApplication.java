package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;

@TeleOp(name = "Teleop App", group = "SA_FTC")
public class TeleopApplication extends OpMode {
    enum ExtensionState {
        EXTENDED,
        RETRACTED
    }

    public static TeleopApplication Instance;

    public MecanumDrive drive;
    Intake intake;
    Outtake outtake;

    MovementUtils movementUtils;

    ExtensionState intakeState = ExtensionState.RETRACTED;
    ExtensionState outtakeState = ExtensionState.RETRACTED;

    @Override
    public void init() {
        Instance = this;

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        movementUtils = new MovementUtils(hardwareMap);
    }

    @Override
    public void loop() {
        movementUtils.fieldCentricMovement();

        // Intake
        if (gamepad2.a) {
            if (intakeState == ExtensionState.RETRACTED) {
                intakeState = ExtensionState.EXTENDED;
                Actions.runBlocking(intake.extend());
            } else {
                intakeState = ExtensionState.RETRACTED;
                Actions.runBlocking(intake.retract());
            }
        }

        // Outtake
        if (gamepad2.y) {
            if (outtakeState == ExtensionState.RETRACTED) {
                outtakeState = ExtensionState.EXTENDED;
                Actions.runBlocking(outtake.extend());
            } else {
                outtakeState = ExtensionState.RETRACTED;
                Actions.runBlocking(outtake.retract());
            }
        }

        // Claw
        if (gamepad2.right_bumper) {
            Actions.runBlocking(intake.closeClaw());
        } else if (gamepad2.left_bumper) {
            Actions.runBlocking(intake.openClaw());
        }

        // Wrist
        if (gamepad2.right_trigger > 0.1) {
            Actions.runBlocking(intake.extendWrist());
        } else if (gamepad2.left_trigger > 0.1) {
            Actions.runBlocking(intake.retractWrist());
        }

        // Bucket
        if (gamepad2.x) {
            Actions.runBlocking(outtake.dunk());
        } else if (gamepad2.b) {
            Actions.runBlocking(outtake.hold());
        }

        // Debugging
        telemetry.update();
    }
}