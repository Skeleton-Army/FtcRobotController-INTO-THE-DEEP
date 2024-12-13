package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.TeleopApplication;
import org.firstinspires.ftc.teamcode.utils.general.Debounce;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;

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
        telemetry.addData("Current Pose", drive.pose);

        if (Debounce.isButtonPressed("a", gamepad1.a)) {
            savedPoses.add(drive.pose);
        }

        for (int i = 0; i < savedPoses.size(); i++) {
            telemetry.addData(String.valueOf(i), savedPoses.get(i));
        }

        super.loop();
    }
}
