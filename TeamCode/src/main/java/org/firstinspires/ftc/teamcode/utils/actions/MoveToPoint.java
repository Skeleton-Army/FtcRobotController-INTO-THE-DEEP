package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagPipeline;

public class MoveToPoint implements Action {
    Pose2d targetPose;

    Follower follower;

    public MoveToPoint(Pose2d targetPose, Follower follower) {
        this.targetPose = targetPose;
        this.follower = follower;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        /*Actions.runBlocking(

        );*/
        return false;
    }
}
