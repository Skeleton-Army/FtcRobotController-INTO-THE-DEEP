package org.firstinspires.ftc.teamcode.utils.actionClasses;

import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pickupInterval;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pickupIntervalDivision;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pickupTimeout;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pixelThreshMaxX;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pixelThreshMaxY;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pixelThreshMinX;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.pixelThreshMinY;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actions.LoopAction;
import org.firstinspires.ftc.teamcode.utils.actions.MoveApriltag;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagSamplesPipeline;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.opencv.core.Point;

//TODO: do stuff here sometime!!
public class Drive {

    MecanumDrive drive;

    AprilTagSamplesPipeline aprilTagSamplesPipeline;
    HardwareMap hardwareMap;
    WebcamCV camCV;

    public Drive(MecanumDrive drive, AprilTagSamplesPipeline aprilTagSamplesPipeline) {
        this.drive = drive;
        this.aprilTagSamplesPipeline = aprilTagSamplesPipeline;
    }

    public Drive(MecanumDrive drive, WebcamCV camCV) {
        this.drive = drive;
        this.camCV = camCV;
    }

    public Action moveApriltag(Pose2d targetPose) {
        return new MoveApriltag(targetPose, drive, aprilTagSamplesPipeline);

    }

    public Action alignToSample(Vector2d targetSamplePos) {
        return getTrajectoryToSample(targetSamplePos);
    }

    public Action alignToSampleContinuous(Sample targetSample) {
        Vector2d targetSamplePos = targetSample.getSamplePosition().position;

        return new LoopAction(
                () -> alignToSample(camCV.getBestSamplePos(targetSamplePos).position),
                () -> new InstantAction(() -> {
                    camCV.resetSampleList();
                    camCV.lookForSamples();
                }),
                () -> new InstantAction(() -> {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                }),
                pickupInterval,
                pickupIntervalDivision,
                pickupTimeout,
                () -> {
                    Point lowest = camCV.getBestSample(targetSamplePos).lowest;
                    return lowest.x > pixelThreshMinX && lowest.x < pixelThreshMaxX && lowest.y > pixelThreshMinY && lowest.y < pixelThreshMaxY;
                }
        );
    }

    public Action alignToSampleVel(Vector2d targetSamplePos) {
        Vector2d offset = new Vector2d(CameraConfig.pickupSampleOffsetX, CameraConfig.pickupSampleOffsetY);
        Vector2d target = targetSamplePos.minus(offset);
        double firstDist = target.norm();

        return new LoopAction(
                () -> new InstantAction(() -> drive.setDrivePowers(getVelocityToSample(camCV.getBestSamplePos(targetSamplePos).position, firstDist))),
                () -> new InstantAction(() -> {
                    camCV.resetSampleList();
                    camCV.lookForSamples();
                }),
                () -> new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))),
                0.2,
                1,
                2
        );
    }

    private PoseVelocity2d getVelocityToSample(Vector2d targetSamplePos, double firstDist) {
        // Assuming the vector is (sampleX, sampleY) sample position relative to the robot

        Vector2d offset = new Vector2d(CameraConfig.pickupSampleOffsetX, CameraConfig.pickupSampleOffsetY);
        Vector2d target = targetSamplePos.minus(offset);

        Vector2d velocity = target.div(firstDist);

        return new PoseVelocity2d(velocity, 0);
    }

    private Action getTrajectoryToSample(Vector2d targetSamplePos) {
        double heading = drive.pose.heading.toDouble();

        Vector2d offset = new Vector2d(
                CameraConfig.pickupSampleOffsetY * Math.cos(heading) - CameraConfig.pickupSampleOffsetX * Math.sin(heading),
                CameraConfig.pickupSampleOffsetY * Math.sin(heading) + CameraConfig.pickupSampleOffsetX * Math.cos(heading)
        );

        Vector2d target = targetSamplePos.minus(offset);

        return drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(target)
                .build();
    }
}
