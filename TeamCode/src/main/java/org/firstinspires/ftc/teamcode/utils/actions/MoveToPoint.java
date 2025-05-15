package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagPipeline;

public class MoveToPoint implements Action {
    Pose2d targetPose;

    MecanumDrive drive;

    public MoveToPoint(Pose2d targetPose, MecanumDrive drive) {
        this.targetPose = targetPose;
        this.drive = drive;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(targetPose, Math.toRadians(225))
                        .build()
        );
        return false;
    }
}
