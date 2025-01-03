package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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


    public PickupSample(Intake intake, Drive actionsDrive) {
        this.intake = intake;
        this.actionsDrive = actionsDrive;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Actions.runBlocking(
            new SequentialAction(
                    // the robot's detecting the sample, and moving to intake position
                    actionsDrive.alignToSample(),
                    // doing the intake part which collects the sample
                    intake.extend(),
                    new SleepAction(0.2),
                    intake.extendWrist(),
                    new SleepAction(0.2),
                    intake.closeClaw(),
                    new SleepAction(0.2),
                    intake.retract(),
                    intake.retractWrist(),
                    new SleepAction(0.2),
                    intake.openClaw()
            )
        );

        return false;
    }
}
