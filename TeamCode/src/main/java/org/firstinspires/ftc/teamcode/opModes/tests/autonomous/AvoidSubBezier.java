package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BezierToPoint")
public class AvoidSubBezier extends OpMode {
    Follower follower;
    Pose beginPos = new Pose(AvoidSubParameters.startX, AvoidSubParameters.startY, Math.toRadians(AvoidSubParameters.startAngle));
    Pose targetPos = new Pose(AvoidSubParameters.endX, AvoidSubParameters.endY, Math.toRadians(AvoidSubParameters.endAngle));

    Obstacle SubObstacle = new Obstacle(48, 96, 48, 48);
    BezierToPoint Bezier;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(beginPos);
        Pose[] controlPoints = new Pose[3];

        controlPoints[0] = beginPos;
        controlPoints[1] = new Pose((beginPos.getX() + targetPos.getX()) / 2,(beginPos.getY() + targetPos.getY()) / 2);
        controlPoints[2] = targetPos;

        List<Obstacle> obstacles = new ArrayList<>();
        obstacles.add(SubObstacle);

        Bezier = new BezierToPoint(controlPoints,3 , obstacles);

        follower.followPath(Bezier.pathchain);
    }

    @Override
    public void init_loop() {
        telemetry.addData("start pos: ", beginPos.getVector());

        telemetry.addLine("------- mid point selected -------");
        telemetry.addData("mid point: ", Bezier.generatedControls[1].getVector());
        telemetry.addLine("----------------------------------");

        telemetry.addData("end pos: ", targetPos.getVector());

        telemetry.update();
    }

    @Override
    public void loop() {

        if (follower.atParametricEnd()) {
            follower.followPath(JuicyBezier.GeneratedPath.paths);
        }
    }
}



