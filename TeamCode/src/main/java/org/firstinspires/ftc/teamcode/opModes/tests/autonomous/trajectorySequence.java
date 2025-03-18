package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;

@Autonomous
public class trajectorySequence extends OpMode {
    MecanumDrive drive;
    Intake intake;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        runBlocking(new SequentialAction(
                intake.extend(),
                new SleepAction(2)
        ));

        // runs intake action 5 UNITS before the spline ends
        runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterDisp(30 - 5, intake.extendWrist()) // 5 inches before
                        .splineTo(new Vector2d(30, 0), 0)
                        .build()
        );

//        runBlocking(new SequentialAction(
//                new SleepAction(2),
//                intake.retractWrist()
//        ));
//
//        // runs intake action after 0.5 sec the spline begins
//        runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .afterTime(1, intake.extendWrist())
//                        .splineTo(new Vector2d(30, 0), 0)
//                        .build()
//        );
    }

    @Override
    public void loop() {}
}
