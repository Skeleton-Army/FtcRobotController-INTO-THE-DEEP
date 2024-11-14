package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Auto Test", group = "SA_FTC")
public class AutoTest extends OpMode {
    MecanumDrive drive;
    Pose2d beginPose = new Pose2d(-48.34, 59.33, Math.toRadians(-90));

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, beginPose);
    }

    @Override
    public void start() {
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(0.09, -47.78), Math.toRadians(0))
                        .splineTo(new Vector2d(47.78, 0.09), Math.toRadians(90))
                        .splineTo(new Vector2d(0.09, 48.16), Math.toRadians(180.00))
                        .splineTo(new Vector2d(-47.97, -0.28), Math.toRadians(270.00))
                        .splineTo(new Vector2d(0.28, -47.97), Math.toRadians(0.00))
                        .splineTo(new Vector2d(47.41, -0.09), Math.toRadians(90.00))
                        .splineTo(new Vector2d(0.28, 47.97), Math.toRadians(180.00))
                        .splineTo(new Vector2d(-48.16, 48.34), Math.toRadians(270.00))
                        .build()
        );
    }

    @Override
    public void loop() {

    }
}
