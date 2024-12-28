package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actions.MoveApriltag;

//TODO: do stuff here sometime!!
public class Drive {

    MecanumDrive drive;

    HardwareMap hardwareMap;
    public Drive(MecanumDrive drive) {
        this.drive = drive;
    }

    public Action moveApriltag(Pose2d targetPose) {
        return new MoveApriltag(targetPose, drive);

    }

}
