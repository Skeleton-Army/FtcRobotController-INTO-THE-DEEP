package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;

import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;

public class AlignToSample implements Action {
    Follower follower;
    Pose targetSamplePos;

    public AlignToSample(Follower follower, Pose targetSamplePos) {
        this.follower = follower;
        this.targetSamplePos = targetSamplePos;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            double heading = follower.getPose().getHeading();

            Pose offset = new Pose(CameraConfig.pickupSampleOffsetY* Math.cos(heading) -
                    CameraConfig.pickupSampleOffsetX * Math.sin(heading),
                 CameraConfig.pickupSampleOffsetY * Math.sin(heading) +
                    CameraConfig.pickupSampleOffsetX * Math.cos(heading));

            targetSamplePos.subtract(offset);

            telemetryPacket.addLine(targetSamplePos.toString());

            Actions.runBlocking(
                    new FollowPath(follower,
                            follower.pathBuilder()
                                    .addPath(new BezierLine(follower.getPose(), targetSamplePos))
                                    .setConstantHeadingInterpolation(heading)
                                    .build()
                    )
            );
        }

        catch (Exception e) {
            telemetryPacket.addLine(e.toString());
        }

        return false;
    }
}
