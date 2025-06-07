package org.firstinspires.ftc.teamcode.utils.actionClasses;

import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pickupInterval;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pickupIntervalDivision;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pickupMinInterval;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pickupSpeed;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pickupTimeout;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.actions.FollowPath;
import org.firstinspires.ftc.teamcode.utils.actions.LoopAction;
import org.firstinspires.ftc.teamcode.utils.actions.MoveApriltag;
import org.firstinspires.ftc.teamcode.utils.actions.MoveToPoint;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.opencv.core.Point;

import java.util.concurrent.atomic.AtomicReference;

public class Drive {
    public static Sample targetSampleStatic;

    Follower follower;
    Apriltag apriltag;

    AprilTagPipeline aprilTagPipeline;
    HardwareMap hardwareMap;

    WebcamCV camCV;
    Telemetry telemetry;

    public Drive(Follower follower, Apriltag apriltag) {
        this.follower = follower;
        this.apriltag = apriltag;
    }

    public Drive(Follower follower) {
        this.follower = follower;
    }

    public Drive(Follower follower, WebcamCV camCV) {
        this.follower = follower;
        this.camCV = camCV;
    }

    public Drive(Follower follower, WebcamCV camCV, Telemetry telemetry) {
        this.follower = follower;
        this.camCV = camCV;
        this.telemetry = telemetry;
    }

    public Action MoveToPoint(Pose targetPose) {
        return new MoveToPoint(targetPose, follower);
    }

    public Action alignToSample(Pose targetSamplePos) {
        return new ParallelAction(
                new InstantAction(() -> targetSampleStatic = camCV.getBestSample(targetSamplePos)),
                getTrajectoryToSample(targetSamplePos)
        );
    }

    public Action alignToSample(Sample targetSample) {
        return alignToSample(targetSample.getSamplePosition());
    }

    public Action alignToSampleContinuous(Sample targetSample) {
        return alignToSampleContinuous(targetSample, new Pose(-1000, -1000), new Pose(1000, 1000));
    }

    public Action alignToSampleContinuous(Sample targetSample, Pose lower, Pose upper) {
        AtomicReference<Pose> targetSamplePos = new AtomicReference<>(targetSample.getSamplePosition());
        return new LoopAction(
                () -> alignToSample(targetSamplePos.get()),
                () -> new InstantAction(() -> {
//                    camCV.resetSampleList();
//                    if (camCV.lookForSamples())
//                        targetSamplePos.set(camCV.getBestSamplePos(targetSamplePos.get()).position);
                }),
                () -> new InstantAction(() -> {
                    follower.breakFollowing();
                }),
                pickupInterval,
                pickupIntervalDivision,
                pickupMinInterval,
                pickupTimeout,
                () -> {
                    if (camCV.lookForSamples()) {
                        Sample newSample = camCV.getBestSampleInRange(targetSamplePos.get(), lower, upper);
                        targetSamplePos.set(newSample.getSamplePosition());

                        targetSampleStatic = newSample;
                    }

                    Point center = targetSampleStatic.center;
                    double dist = Math.sqrt(Math.pow(center.x - CameraConfig.pixelOptimalCenterX, 2) + Math.pow(center.y - CameraConfig.pixelOptimalCenterY, 2));

//                    telemetry.addData("dist", dist);
//                    telemetry.update();

                    return dist <= CameraConfig.pixelThreshRadius;
                }
        );
    }

    private Action getTrajectoryToSample(Pose targetSamplePos) {
        double heading = follower.getPose().getHeading();

        Pose offset = new Pose(
                CameraConfig.pickupSampleOffsetY * Math.cos(heading) - CameraConfig.pickupSampleOffsetX * Math.sin(heading),
                CameraConfig.pickupSampleOffsetY * Math.sin(heading) + CameraConfig.pickupSampleOffsetX * Math.cos(heading)
        );

        targetSamplePos.subtract(offset);

        return new FollowPath(follower,
                follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), targetSamplePos))
                    .setConstantHeadingInterpolation(heading)
                    .build()
        );
    }
}
