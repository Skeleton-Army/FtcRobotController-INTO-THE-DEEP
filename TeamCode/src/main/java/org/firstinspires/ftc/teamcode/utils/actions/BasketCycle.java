package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;

import java.util.Objects;

public class BasketCycle implements Action {

    Drive Actionsdrive;

    Apriltag apriltag;

    Outtake outtake;

    Pose dunkPose;

    Action extendOuttake;

    Action dunk;
    Action dunkSequence;
    public BasketCycle(Drive Actionsdrive, Outtake outtake, String alliance) {
        this.Actionsdrive = Actionsdrive;
        this.outtake = outtake;

        if (Objects.equals(alliance, "blue")) {
            dunkPose = new Pose(51,54,Math.toRadians(225), false);
        }
        else if (Objects.equals(alliance, "red")) {
            dunkPose = new Pose(-56,-54.5,Math.toRadians(45), false);
        }

        extendOuttake = new ParallelAction(
                outtake.bucketMiddle(),
                outtake.extend(),
                new SequentialAction(
                        new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -400),
                        outtake.bucketReady()
                )
        );

        dunk = new SequentialAction(
                new SleepUntilAction(() -> outtake.motor.getCurrentPosition() < -850),
                outtake.dunk(),
                new SleepAction(0.25)
        );

        dunkSequence = new ParallelAction(
                extendOuttake,
                dunk
        );

    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                Actionsdrive.MoveToPoint(dunkPose), dunkSequence)
        ));
        return false;
    }
}
