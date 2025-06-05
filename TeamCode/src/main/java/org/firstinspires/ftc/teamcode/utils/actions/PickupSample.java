package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

public class PickupSample implements Action {
    Drive actionsDrive;
    Intake intake;
    Pose targetSamplePos;


    public PickupSample(Intake intake, Drive actionsDrive, Vector2d targetSamplePos) {
        this.intake = intake;
        this.actionsDrive = actionsDrive;
        this.targetSamplePos = targetSamplePos;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Actions.runBlocking(
                new ParallelAction(
                        actionsDrive.alignToSample(targetSamplePos),
                        new SequentialAction(
                                // the robot's detecting the sample, and moving to intake position
                                intake.openClaw(),
                                intake.wristReady(),
                                intake.extend(),
                                intake.extendWrist(),
                                new SleepAction(0.4),
                                intake.closeClaw(),
                                new SleepAction(0.4),
                                intake.retractWrist(),
                                new ParallelAction(
                                        intake.retract(),
                                        new SleepAction(0.4)
                                ),
                                intake.openClaw(),
                                intake.wristMiddle()
                        )
                )
        );

        return false;
    }
}
