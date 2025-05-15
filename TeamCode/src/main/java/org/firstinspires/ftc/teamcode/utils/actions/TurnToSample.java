package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;

public class TurnToSample implements Action {
    final MecanumDrive drive;
    final Intake intake;
    final Sample targetSample;
    public TurnToSample(MecanumDrive drive, Intake intake, Sample targetSample) {
        this.drive = drive;
        this.intake = intake;
        this.targetSample = targetSample;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            Vector2d currPose = drive.pose.position;
            Vector2d sampleVec = targetSample.getSamplePosition().position;
            Vector2d relative = sampleVec.minus(currPose);
            double distance = relative.norm();
            double targetAngle = Math.atan2(relative.y, relative.x) + Math.acos(IntakeConfig.offsetFromCenterX / distance) - Math.PI / 2;

            double orientation = Math.toDegrees(targetAngle) - targetSample.orientation;
//            telemetryPacket.addLine(orientation + "");
            double normalizedOrientation = (90 - Math.abs(orientation)) * Math.signum(orientation);
//            telemetryPacket.addLine(normalizedOrientation + "");
            double rotationTarget = normalizedOrientation / 90;
//            telemetryPacket.addLine(rotationTarget + "");
//            telemetryPacket.addLine(Utilities.remap(0.5, -1, 0, 1, IntakeConfig.rotationLeft, IntakeConfig.rotationForward, IntakeConfig.rotationRight) + "");
//            telemetryPacket.addLine(sampleVec.toString());

            Actions.runBlocking(
                    intake.rotate(rotationTarget)
            );
            Actions.runBlocking(new SleepAction(0.5));

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                            .turnTo(targetAngle)
                            .build()
            );
        }

        catch (Exception e) {
            telemetryPacket.addLine(e.toString());
        }

        return false;
    }
}
