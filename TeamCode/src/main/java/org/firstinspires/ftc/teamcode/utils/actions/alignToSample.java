package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

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


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(targetSamplePos.x, targetSamplePos.y - 11), drive.pose.heading.toDouble())
                            .build()
            );
        }

        catch (Exception e) {
            telemetryPacket.addLine(e.toString());
        }

        return false;
    }
}
