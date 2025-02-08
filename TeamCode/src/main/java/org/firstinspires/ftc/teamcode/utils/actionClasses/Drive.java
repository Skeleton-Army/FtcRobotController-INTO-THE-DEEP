package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actions.MoveApriltag;
import org.firstinspires.ftc.teamcode.utils.actions.alignToSample;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagSamplesPipeline;

//TODO: do stuff here sometime!!
public class Drive {

    MecanumDrive drive;

    AprilTagSamplesPipeline aprilTagSamplesPipeline;
    HardwareMap hardwareMap;
    public Drive(MecanumDrive drive, AprilTagSamplesPipeline aprilTagSamplesPipeline) {
        this.drive = drive;
        this.aprilTagSamplesPipeline = aprilTagSamplesPipeline;
    }

    public Drive(MecanumDrive drive) {
        this.drive = drive;
    }

    public Action moveApriltag(Pose2d targetPose) {
        return new MoveApriltag(targetPose, drive, aprilTagSamplesPipeline);

    }

    public Action alignToSample(Vector2d targetSamplePos) {
        return new alignToSample(drive, targetSamplePos);
    }

}
