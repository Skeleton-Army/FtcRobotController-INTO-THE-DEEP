package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;

public class PickupSample implements Action {

    Drive actionsDrive;
    Intake intake;
    Sample targetSample;


    public PickupSample(Intake intake, Drive actionsDrive, Sample targetSample) {
        this.intake = intake;
        this.actionsDrive = actionsDrive;
        this.targetSample = targetSample;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Actions.runBlocking(
                new ParallelAction(
                        actionsDrive.alignToSampleContinuous(targetSample),
                        new SequentialAction(
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
