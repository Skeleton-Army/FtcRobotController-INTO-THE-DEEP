package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;

import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class MoveApriltag implements Action {
    Apriltag apriltag;
    Pose targetPose;

    Follower follower;

    public MoveApriltag(Pose targetPose, Follower follower, Apriltag apriltag) {
        this.targetPose = targetPose;
        this.follower = follower;
        this.apriltag = apriltag;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        apriltag.enableApriltag();
        AprilTagDetection detections = apriltag.getCurrentDetections().get(0);

        if (detections != null) {

            // if the target position is the origin, the target position will be the Apriltag's position on the field
            if (targetPose.equals(new Pose(0,0,0)))
                this.targetPose = new Pose(detections.rawPose.x - 6, detections.rawPose.y - 6,0);


            // do a spline to the target apriltag, in this case the first one that was detected
            Actions.runBlocking(
                    new FollowPath(follower,
                            follower.pathBuilder()
                                    .addPath(new BezierLine(apriltag.getRobotPos(detections), targetPose))
                                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), 0)
                                    .build()
                    )
            );

        }
        return false;
    }
}
