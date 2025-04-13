package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class newFollower extends OpMode {

    MecanumDrive drive;
    Pose2d beginPose = new Pose2d(0,0,0);

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, beginPose);
    }

    @Override
    public void start() {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(32,32), Math.toRadians(45))
                .build()
        );


    }

    @Override
    public void loop() {

    }
}
