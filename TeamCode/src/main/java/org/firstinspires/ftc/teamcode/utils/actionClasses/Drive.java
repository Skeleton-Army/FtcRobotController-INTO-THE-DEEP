package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actions.FailoverAction;
import org.firstinspires.ftc.teamcode.utils.actions.LoopAction;
import org.firstinspires.ftc.teamcode.utils.actions.MoveApriltag;
import org.firstinspires.ftc.teamcode.utils.actions.DynamicAction;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;

public class Drive {
    MecanumDrive drive;
    Apriltag apriltag;
    WebcamCV camCV;

    public Drive(MecanumDrive drive, Apriltag apriltag) {
        this.drive = drive;
        this.apriltag = apriltag;
    }

    public Drive(MecanumDrive drive) {
        this.drive = drive;
    }

    public Drive(MecanumDrive drive, WebcamCV camCV) {
        this.drive = drive;
        this.camCV = camCV;
    }

    public Action moveApriltag(Pose2d targetPose) {
        return new MoveApriltag(targetPose, drive, apriltag);
    }

    public Action alignToSample(Vector2d targetSamplePos) {
        return getTrajectoryToSample(targetSamplePos);
    }

    public Action alignToSampleContinuous(Vector2d targetSamplePos) {
        return new LoopAction(
                () -> alignToSample(camCV.getBestSamplePos(targetSamplePos).position),
                () -> new InstantAction(() -> {
                    camCV.resetSampleList();
                    camCV.lookForSamples();
                }),
                () -> new InstantAction(() -> {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                }),
                0.5,
                2 // How much time to wait until the continuous alignment ends
        );

//        FailoverAction align = new FailoverAction(
//                new DynamicAction(() -> alignToSample(camCV.getBestSamplePos(targetSamplePos).position)),
//
//                new SequentialAction(
//                        new InstantAction(() -> {
//                            camCV.resetSampleList();
//                            camCV.lookForSamples();
//                        }),
//                        new DynamicAction(() -> alignToSample(camCV.getBestSamplePos(targetSamplePos).position))
//                )
//        );
//
//        return new ParallelAction(
//                align,
//
//                new SequentialAction(
//                        new SleepAction(1),
//                        new InstantAction(align::failover)
//                )
//        );
    }

    private Action getTrajectoryToSample(Vector2d targetSamplePos) {
        double heading = drive.pose.heading.toDouble();

        Vector2d offset = new Vector2d(
                CameraConfig.pickupSampleOffsetY * Math.cos(heading) - CameraConfig.pickupSampleOffsetX * Math.sin(heading),
                CameraConfig.pickupSampleOffsetY * Math.sin(heading) + CameraConfig.pickupSampleOffsetX * Math.cos(heading)
        );

        Vector2d target = targetSamplePos.minus(offset);

//        Vector2d moveDirection = targetSamplePos.minus(drive.pose.position).div(4); // Smooth movement
//        drive.setDrivePowers(new PoseVelocity2d(moveDirection, 0)); // Move toward sample

        return drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(target)
                    .build();
    }
}
