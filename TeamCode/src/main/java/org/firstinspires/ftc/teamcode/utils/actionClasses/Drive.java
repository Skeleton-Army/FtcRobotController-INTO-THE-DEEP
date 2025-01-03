package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actions.MoveApriltag;
import org.firstinspires.ftc.teamcode.utils.actions.alignToSample;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;

//TODO: do stuff here sometime!!
public class Drive {

    MecanumDrive drive;

    Apriltag apriltag;
    HardwareMap hardwareMap;
    DetectSamples detectSamples;
    public Drive(MecanumDrive drive, Apriltag apriltag, DetectSamples detectSamples) {
        this.drive = drive;
        this.apriltag = apriltag;
        this.detectSamples = detectSamples;
    }

    public Action moveApriltag(Pose2d targetPose) {
        return new MoveApriltag(targetPose, drive, apriltag);

    }

    public Action alignToSample() {
        return new alignToSample(drive, detectSamples);
    }

}
