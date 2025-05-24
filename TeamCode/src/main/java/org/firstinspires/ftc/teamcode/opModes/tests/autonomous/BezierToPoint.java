package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.Arrays;
import java.util.List;

public class BezierToPoint {

    Pose[] generatedControls;
    public PathChain pathchain;

    public BezierToPoint(Pose[] controlPoints, int numPoints, List<Obstacle> obstacles) {
        Pose[] safeControlPoints = avoidRectangularObstacles(controlPoints, obstacles, numPoints);
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
        System.arraycopy(points, 0, temp, 0, n);

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

    private static Pose[] avoidRectangularObstacles(Pose[] controlPoints, List<Obstacle> obstacles, int numSamples) {
        Pose[] adjusted = Arrays.copyOf(controlPoints, controlPoints.length);
        final double stepSize = 2.0;
        final int maxIterations = 20;

        for (int iteration = 0; iteration < maxIterations; iteration++) {
            boolean collisionFound = false;
            Pose[] pathSamples = computeBezierPoints(adjusted, numSamples);

            for (Pose p : pathSamples) {
                for (Obstacle obs : obstacles) {
                    if (obs.isColliding(p, AvoidSubParameters.width, AvoidSubParameters.height)) {
                        collisionFound = true;

                        // Adjust mid control points only
                        for (int i = 1; i < adjusted.length - 1; i++) {
                            Pose mid = adjusted[i];
                            double dx = mid.getVector().getXComponent() - (obs.x + obs.width / 2.0);
                            double dy = mid.getVector().getYComponent() - (obs.y + obs.height / 2.0);
                            double norm = Math.sqrt(dx * dx + dy * dy);
                            if (norm == 0) norm = 1;
                            dx /= norm;
                            dy /= norm;

                            double newX = mid.getVector().getXComponent() + dx * stepSize;
                            double newY = mid.getVector().getYComponent() + dy * stepSize;

                            adjusted[i] = new Pose(newX, newY, mid.getHeading());
                        }

                        break;
                    }
                }
                if (collisionFound) break;
            }

            if (!collisionFound) break;
        }

        return adjusted;
    }
}
