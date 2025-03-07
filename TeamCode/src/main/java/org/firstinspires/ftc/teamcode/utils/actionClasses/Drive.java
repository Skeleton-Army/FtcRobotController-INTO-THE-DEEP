package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actions.ConditionAction;
import org.firstinspires.ftc.teamcode.utils.actions.MoveApriltag;
import org.firstinspires.ftc.teamcode.utils.actions.AlignToSample;
import org.firstinspires.ftc.teamcode.utils.actions.RaceAction;
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
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
//        return new AlignToSample(drive, targetSamplePos);
        return getTrajectoryToSample(targetSamplePos);
    }

    public Action alignToSampleContinuous(Vector2d targetSamplePos) {
        return new RaceAction(
                new ConditionAction(
                        new ParallelAction(
                                new SleepUntilAction(() -> camCV.lookForSamples()),
                                alignToSample(camCV.getBestSamplePos(targetSamplePos).position)
                        ),
                        () -> true // Repeatedly call the action (until the sleep has completed)
                ),
                new SleepAction(1) // How much time to wait until it ends the continuous alignment
        );
    }

    private Action getTrajectoryToSample(Vector2d targetSamplePos) {
        double heading = drive.pose.heading.toDouble();

        Vector2d offset = new Vector2d(
                CameraConfig.pickupSampleOffsetY * Math.cos(heading) - CameraConfig.pickupSampleOffsetX * Math.sin(heading),
                CameraConfig.pickupSampleOffsetY * Math.sin(heading) + CameraConfig.pickupSampleOffsetX * Math.cos(heading)
        );

        Vector2d target = targetSamplePos.minus(offset);

        return drive.actionBuilder(drive.pose).fresh()
                    .splineToConstantHeading(target, drive.pose.heading)
                    .build();
    }
}
