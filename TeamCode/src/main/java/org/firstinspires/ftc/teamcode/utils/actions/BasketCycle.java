package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;

import java.util.Objects;

public class BasketCycle implements Action {

    Drive Actionsdrive;

    Apriltag apriltag;

    Outtake outtake;

    Pose2d dunkPose;

    public BasketCycle(Drive Actionsdrive, Outtake outtake, String alliance) {
        this.Actionsdrive = Actionsdrive;
        this.outtake = outtake;

        if (Objects.equals(alliance, "blue")) {
            dunkPose = new Pose2d(51,54,Math.toRadians(225));
        }
        else if (Objects.equals(alliance, "red")) {
            dunkPose = new Pose2d(-51,-54,Math.toRadians(45));
        }
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Actions.runBlocking(new SequentialAction(
                Actionsdrive.moveApriltag(dunkPose),
                outtake.extend(),
                new SleepAction(0.5),
                outtake.dunk(),
                new SleepAction(0.2),
                outtake.hold(),
                outtake.retract()
        ));
        return false;
    }
}
