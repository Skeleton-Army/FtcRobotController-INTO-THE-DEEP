package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;

@Autonomous(name = "TestActions", group = "SA_FTC")
public class ActionsTest extends OpMode {
    MecanumDrive drive;
    Intake intake;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        Actions.runBlocking(intake.motorToPosition(-2700, 1));
    }

    @Override
    public void loop() {

    }
}
