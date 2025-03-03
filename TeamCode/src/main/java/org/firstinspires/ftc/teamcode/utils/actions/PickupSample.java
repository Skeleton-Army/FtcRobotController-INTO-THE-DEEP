package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

public class PickupSample implements Action {

    Drive actionsDrive;
    Intake intake;
    Vector2d targetSamplePos;


    public PickupSample(Intake intake, Drive actionsDrive, Vector2d targetSamplePos) {
        this.intake = intake;
        this.actionsDrive = actionsDrive;
        this.targetSamplePos = targetSamplePos;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Actions.runBlocking(
            new SequentialAction(
                    // the robot's detecting the sample, and moving to intake position
                    actionsDrive.alignToSample(targetSamplePos),
                    new SleepAction(0.1),
                    // doing the intake part which collects the sample
                    intake.openClaw(),
                    new SleepAction(0.1),
                    intake.extend(),
                    new ParallelAction(
                            intake.extendWrist(),
                            new SleepAction(0.2)
                    ),
                    new SleepAction(1),
                    intake.closeClaw(),
                    new SleepAction(0.6),
                    intake.retractWrist(),
                    new SleepAction(0.6),
                    intake.retract(),
                    new SleepAction(0.2),
                    intake.openClaw(),
                    new SleepAction(0.1),
                    intake.wristMiddle()
            )
        );

        return false;
    }
}
