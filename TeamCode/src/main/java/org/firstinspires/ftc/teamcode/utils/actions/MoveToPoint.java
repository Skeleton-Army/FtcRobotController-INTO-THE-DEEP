package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.BezierToPoint;

public class MoveToPoint implements Action {
    Pose targetPose;

    Follower follower;
    BezierToPoint bezier;
    Telemetry telemetry;

    public MoveToPoint(Pose targetPose, Follower follower, Telemetry telemetry) {
        this.targetPose = targetPose;
        this.follower = follower;
        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        bezier = new BezierToPoint(follower.getPose(), new Pose(-55, -55, Math.toRadians(45), false),true, telemetry);
        PathChain path = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(follower.getPose().getX(),follower.getPose().getY(),Point.CARTESIAN),
                                new Point(bezier.midPoint.x,bezier.midPoint.y,Point.CARTESIAN),
                                new Point(bezier.endPose.getX(),bezier.endPose.getY(),Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(bezier.endPose.getHeading(), bezier.endPose.getHeading())
                .build();
        Actions.runBlocking(
                new InstantAction(() -> follower.followPath(path))
        );
        return false;
    }
}
