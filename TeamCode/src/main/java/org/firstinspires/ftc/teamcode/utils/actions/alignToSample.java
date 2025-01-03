package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;

public class alignToSample implements Action {

    MecanumDrive drive;
    DetectSamples detectSamples;
    public alignToSample(MecanumDrive drive, DetectSamples detectSamples) {
        this.drive = drive;
        this.detectSamples = detectSamples;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            Sample targetSample = Utilities.calculateClosest(detectSamples);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(targetSample.getSampleY(), targetSample.getSampleX() - 11), drive.pose.heading.toDouble())
                            .build()
            );
        }

        catch (Exception e) {
            telemetryPacket.addLine(e.toString());
        }

        return false;
    }
}
