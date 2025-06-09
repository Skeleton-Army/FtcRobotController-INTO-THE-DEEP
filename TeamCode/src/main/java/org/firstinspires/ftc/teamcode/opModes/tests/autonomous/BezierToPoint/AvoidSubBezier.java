package org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModes.tests.autonomous.JuicyBezier;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BezierToPoint")
public class AvoidSubBezier extends OpMode {
    Follower follower;
    Pose beginPos = new Pose(AvoidSubParametersConfig.startX, AvoidSubParametersConfig.startY, Math.toRadians(AvoidSubParametersConfig.startAngle));
    Pose targetPos = new Pose(AvoidSubParametersConfig.endX, AvoidSubParametersConfig.endY, Math.toRadians(AvoidSubParametersConfig.endAngle));

    Obstacle SubObstacle = new Obstacle(48, 96, 48, 48);
    BezierToPoint2 Bezier;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    PathChain path;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setupConstants(FConstants.class, LConstants.class);
        follower.setStartingPose(beginPos);
        Pose[] controlPoints = new Pose[3];

        /*controlPoints[0] = beginPos;
        controlPoints[1] = new Pose((beginPos.getX() + targetPos.getX()) / 2,(beginPos.getY() + targetPos.getY()) / 2);
        controlPoints[2] = targetPos;*/

        List<Obstacle> obstacles = new ArrayList<>();
        obstacles.add(SubObstacle);

        Bezier = new BezierToPoint2(beginPos, targetPos, true, telemetry);

        path = follower.pathBuilder()
                    .addPath(
                        new BezierCurve(
                            new Point(beginPos.getX(),beginPos.getY(),Point.CARTESIAN),
                            new Point(Bezier.midPoint.x,Bezier.midPoint.y,Point.CARTESIAN),
                            new Point(targetPos.getX(),targetPos.getY(),Point.CARTESIAN)
                        )
                    )
                    .setLinearHeadingInterpolation(beginPos.getHeading(), targetPos.getHeading())
                    .build();

        follower.followPath(path);
    }

    @Override
    public void init_loop() {
        /*for (Pose p : Bezier.generatedControls) {
            packet.fieldOverlay().setStroke("green").strokeCircle(
                    p.getVector().getXComponent(),
                    p.getVector().getYComponent(),
                    2
            );
        }

        // I think it gets the points along the curve, but we will see...
        for (double[] p : Bezier.pathchain.getPath(0).getDashboardDrawingPoints())
        {
            packet.fieldOverlay().setStroke("blue").strokeCircle(
                    p[0],
                    p[1],
                    2
            );
        }*/

        telemetry.addData("start posx: ", beginPos.getVector().getXComponent());
        telemetry.addData("start posy: ", beginPos.getVector().getYComponent());

        telemetry.addLine("------- mid point selected -------");
        /*telemetry.addData("mid pointx: ", Bezier.generatedControls[1].getVector().getXComponent());
        telemetry.addData("mid pointy: ", Bezier.generatedControls[1].getVector().getYComponent());*/
        telemetry.addData("mid pointx: ", Bezier.midPoint.x);
        telemetry.addData("mid pointy: ", Bezier.midPoint.y);
        telemetry.addLine("----------------------------------");

        telemetry.addData("end posx: ", targetPos.getVector().getXComponent());
        telemetry.addData("end posy: ", targetPos.getVector().getYComponent());

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        follower.drawOnDashBoard();
        if (follower.atParametricEnd()) {
            follower.followPath(path);
        }
    }
}



