package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;


import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AvoidSubBezier extends OpMode {
    Pose beginPos;
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    public static PathChain getBezier() {

        PathBuilder builder = new PathBuilder();

        PathChain paths = builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8.000, 80.000, Point.CARTESIAN),
                                new Point(36.000, 80.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        return paths;
    }
}

