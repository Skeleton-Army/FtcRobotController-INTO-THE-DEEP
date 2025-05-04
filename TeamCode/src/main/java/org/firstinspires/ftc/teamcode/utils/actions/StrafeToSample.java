package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.kotlin.extensions.geometry.Vector2dExtKt;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;

public class StrafeToSample implements Action {
    final MecanumDrive drive;
    final Intake intake;
    final Vector2d targetSampleVec;
    public StrafeToSample(MecanumDrive drive, Intake intake, Vector2d targetSampleVec) {
        this.drive = drive;
        this.intake = intake;
        this.targetSampleVec = targetSampleVec;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            Vector2d relative = drive.pose.position.minus(targetSampleVec);
            double distance = relative.norm();

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                            .build()
            );
        }

        catch (Exception e) {
            telemetryPacket.addLine(e.toString());
        }

        return false;
    }
}