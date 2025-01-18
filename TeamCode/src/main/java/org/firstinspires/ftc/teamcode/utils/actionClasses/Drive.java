package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actions.MoveApriltag;
import org.firstinspires.ftc.teamcode.utils.actions.alignToSample;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;

//TODO: do stuff here sometime!!
public class Drive {

    MecanumDrive drive;

    Apriltag apriltag;
    HardwareMap hardwareMap;
    public Drive(MecanumDrive drive, Apriltag apriltag) {
        this.drive = drive;
        this.apriltag = apriltag;
    }

    public Drive(MecanumDrive drive) {
        this.drive = drive;
    }

    public Action moveApriltag(Pose2d targetPose) {
        return new MoveApriltag(targetPose, drive, apriltag);

    }

    public Action alignToSample(Vector2d targetSamplePos) {
        return new alignToSample(drive, targetSamplePos);
    }

}
