package org.firstinspires.ftc.teamcode.utils.actions;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;

public class PickupSample implements Action {

    Drive driveActions;
    Intake intake;
    Sample targetSample;

    double rotationTarget;

    public PickupSample(Intake intake, Drive driveActions, Sample targetSample) {
        this.intake = intake;
        this.driveActions = driveActions;
        this.targetSample = targetSample;

        double orientation = -targetSample.orientation;
        double normalizedOrientation = (90 - Math.abs(orientation)) * Math.signum(orientation);
        rotationTarget = normalizedOrientation / 90;
    }

    Action extendSequence = new SequentialAction(
            intake.wristReady(),
            intake.openClaw(),
            new ParallelAction(
                    intake.extend()
//                        new SequentialAction(
//                                new SleepUntilAction(() -> intake.motor.getCurrentPosition() >= 400),
//                                intake.extendWrist()
//                        )
            )
//                new SleepAction(0.1)
    );

    Action grabSequence = new SequentialAction(
            intake.extendWrist(),
            new SleepAction(0.2),
            intake.closeClaw(),
            new SleepAction(0.1)
//                intake.retractWrist()
//                new SleepAction(0.1)
    );
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        runBlocking(
                intake.rotate(rotationTarget)
        );

        runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveActions.alignToSample(targetSample.getSamplePosition().position),
                                extendSequence
                        ),
                        grabSequence
                )
        );

        return false;
    }
}
