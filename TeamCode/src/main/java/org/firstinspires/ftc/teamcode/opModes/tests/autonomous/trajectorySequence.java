package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;

public class trajectorySequence extends OpMode {
    MecanumDrive drive;
    Intake intake;
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        runBlocking(new SequentialAction(
                intake.extend(),
                new SleepAction(2)
        ));

        // runs intake action 5 UNITS before the spline ends
        drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(15, 0 ), 0)
                .afterDisp(5, intake.extendWrist())
                .build();

        runBlocking(new SequentialAction(
                new SleepAction(2)
        ));

        // runs intake action after 0.5 sec the spline begins
        drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(30, 0 ), 0)
                .afterTime(0.5, intake.extendWrist())
                .build();
    }
}
