package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.TeleopApplication;
import org.firstinspires.ftc.teamcode.utils.general.Debounce;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

/*
    A tool for fetching accurate poses for autonomous trajectories.

    At the start, the robot should be placed with its center at the
    intersection of the tiles in front of the red specimen rungs, facing the rungs.
 */
@TeleOp(name = "Autonomous Pose Tool", group = "SA_FTC")
public class AutonomousPoseTool extends TeleopApplication {
    List<Pose2d> savedPoses = new ArrayList<>();

    @Override
    public void init() {
        PoseStorage.currentPose = new Pose2d(0.00, -48.00, Math.toRadians(90.00));

        super.init();
    }

    @Override
    public void loop() {
        DecimalFormat f = new DecimalFormat("##.0");

        telemetry.addData("Current X", f.format(drive.pose.position.x));
        telemetry.addData("Current Y", f.format(drive.pose.position.y));
        telemetry.addData("Current Heading", f.format(Math.toDegrees(drive.pose.heading.toDouble())));

        if (Debounce.isButtonPressed("a", gamepad1.a)) {
            savedPoses.add(drive.pose);
        }

        for (int i = 0; i < savedPoses.size(); i++) {
            telemetry.addData(i + " X", f.format(savedPoses.get(i).position.x));
            telemetry.addData(i + " Y", f.format(savedPoses.get(i).position.y));
            telemetry.addData(i + " Heading", f.format(Math.toDegrees(savedPoses.get(i).heading.toDouble())));
        }

        telemetry.addLine();

        super.loop();
    }
}
