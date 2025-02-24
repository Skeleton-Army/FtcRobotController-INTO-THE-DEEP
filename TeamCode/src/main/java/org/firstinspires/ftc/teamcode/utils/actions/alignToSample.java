package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;

public class alignToSample implements Action {
    MecanumDrive drive;
    Vector2d targetSamplePos;
    public alignToSample(MecanumDrive drive, Vector2d targetSamplePos) {
        this.drive = drive;
        this.targetSamplePos = targetSamplePos;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            double heading = drive.pose.heading.toDouble();
            Vector2d offset = new Vector2d(CameraConfig.pickupSampleOffsetY* Math.cos(heading) -
                    CameraConfig.pickupSampleOffsetX * Math.sin(heading),
                 CameraConfig.pickupSampleOffsetY * Math.sin(heading) +
                    CameraConfig.pickupSampleOffsetX * Math.cos(heading));
            targetSamplePos = targetSamplePos.minus(offset);

            telemetryPacket.addLine(targetSamplePos.toString());

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(drive.pose.position.x - 1.5, drive.pose.position.y, drive.pose.heading.toDouble()))
                            .splineToConstantHeading(targetSamplePos, heading)
                            .build()
            );
        }

        catch (Exception e) {
            telemetryPacket.addLine(e.toString());
        }

        return false;
    }
}
