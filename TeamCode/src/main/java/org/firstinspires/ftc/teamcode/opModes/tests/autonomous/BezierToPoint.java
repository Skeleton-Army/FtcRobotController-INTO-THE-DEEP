package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;

import java.util.*;

public class BezierToPoint {

    Pose[] generatedControls;
    public PathChain pathchain;

    /*public static class Obstacle {
        public final double x, y, width, height;

        public Obstacle(double x, double y, double width, double height) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
        }

        public boolean isColliding(Pose pose) {
            Vector point = pose.getVector();
            return point.getXComponent() >= x && point.getXComponent() <= x + width &&
                    point.getYComponent() >= y && point.getYComponent() <= y + height;
        }
    }*/

    public BezierToPoint(Pose[] controlPoints, int numPoints, List<Obstacle> obstacles) {
        Pose[] safeControlPoints = avoidRectangularObstacles(controlPoints, obstacles);
        generatedControls = computeBezierPoints(safeControlPoints, numPoints);
        pathchain = generateBezierPathChain(safeControlPoints);
    }

    public static PathChain generateBezierPathChain(Pose[] controlPoints) {
        if (controlPoints == null || controlPoints.length < 2) {
            throw new IllegalArgumentException("At least 2 control points are required for a Bézier curve.");
        }

        Point[] points = new Point[controlPoints.length];
        for (int i = 0; i < controlPoints.length; i++) {
            points[i] = new Point(
                    controlPoints[i].getVector().getXComponent(),
                    controlPoints[i].getVector().getYComponent(),
                    Point.CARTESIAN
            );
        }

        PathBuilder builder = new PathBuilder()
                .addBezierCurve(points)
                .setLinearHeadingInterpolation(
                        controlPoints[0].getHeading(),
                        controlPoints[controlPoints.length - 1].getHeading()
                );

        return builder.build();
    }

    private static Pose[] computeBezierPoints(Pose[] controlPoints, int numPoints) {
        if (controlPoints == null || controlPoints.length < 2) {
            throw new IllegalArgumentException("At least 2 control points are required.");
        }

        Pose[] result = new Pose[numPoints];

        for (int i = 0; i < numPoints; i++) {
            double t = i / (double) (numPoints - 1);
            result[i] = deCasteljau(controlPoints, t);
        }

        return result;
    }

    private static Pose deCasteljau(Pose[] points, double t) {
        int n = points.length;
        Pose[] temp = new Pose[n];
        //System.arraycopy(points, 0, temp, 0, n);

        for(int i = 0; i < n; i++) {
            temp[i] = points[i];
        }

        for (int r = 1; r < n; r++) {
            for (int i = 0; i < n - r; i++) {
                double x = (1 - t) * temp[i].getVector().getXComponent() + t * temp[i + 1].getVector().getXComponent();
                double y = (1 - t) * temp[i].getVector().getYComponent() + t * temp[i + 1].getVector().getYComponent();
                double heading = (1 - t) * temp[i].getHeading() + t * temp[i + 1].getHeading();
                temp[i] = new Pose(x, y, heading);
            }
        }

        return temp[0];
    }

    private static Pose[] avoidRectangularObstacles(Pose[] controlPoints, List<Obstacle> obstacles) {
        Pose[] adjusted = new Pose[controlPoints.length];

        for (int i = 1; i < controlPoints.length - 1; i++) {
            Pose pose = controlPoints[i];
            Pose newPose = pose;

            for (Obstacle obs : obstacles) {
                while (obs.isColliding(newPose)) {
                    double shiftX = (newPose.getVector().getXComponent() > obs.x + obs.width / 2) ? 1 : -1;
                    double shiftY = (newPose.getVector().getYComponent() > obs.y + obs.height / 2) ? 1 : -1;
                    newPose = new Pose(
                                    newPose.getVector().getXComponent() + shiftX,
                                    newPose.getVector().getYComponent() + shiftY
                            ,
                            newPose.getHeading()
                    );
                }
            }

            adjusted[i] = newPose;
        }

        return adjusted;
    }
}
