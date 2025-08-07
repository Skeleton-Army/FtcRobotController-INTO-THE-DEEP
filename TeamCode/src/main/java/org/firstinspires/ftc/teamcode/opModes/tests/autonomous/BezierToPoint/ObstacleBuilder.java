package org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint;

import java.util.ArrayList;
import java.util.List;

public class ObstacleBuilder {

    /**
     * Converts a line segment into a thin rectangle (polygon) for obstacle detection.
     */
    public static double[][] lineToObstacle(double x1, double y1, double x2, double y2, double thickness) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double length = Math.hypot(dx, dy);

        if (length == 0) {
            double half = thickness / 2;
            return new double[][] {
                    {x1 - half, y1 - half},
                    {x1 + half, y1 - half},
                    {x1 + half, y1 + half},
                    {x1 - half, y1 + half}
            };
        }

        dx /= length;
        dy /= length;

        double px = -dy * (thickness / 2);
        double py = dx * (thickness / 2);

        return new double[][] {
                {x1 + px, y1 + py},
                {x1 - px, y1 - py},
                {x2 - px, y2 - py},
                {x2 + px, y2 + py}
        };
    }

    /**
     * Creates a rectangle obstacle centered at (cx, cy) with rotation.
     */
    public static double[][] rectangleObstacle(double cx, double cy, double width, double height, double angle) {
        double dx = width / 2.0;
        double dy = height / 2.0;

        double[][] corners = new double[][] {
                {-dx, -dy},
                { dx, -dy},
                { dx,  dy},
                {-dx,  dy}
        };

        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        for (int i = 0; i < corners.length; i++) {
            double x = corners[i][0];
            double y = corners[i][1];
            double rx = x * cos - y * sin;
            double ry = x * sin + y * cos;
            corners[i][0] = rx + cx;
            corners[i][1] = ry + cy;
        }

        return corners;
    }

    /**
     * Approximates a circular obstacle as a polygon.
     */
    public static double[][] circleObstacle(double centerX, double centerY, double radius, int numSides) {
        List<double[]> points = new ArrayList<>();
        for (int i = 0; i < numSides; i++) {
            double theta = 2 * Math.PI * i / numSides;
            double x = centerX + radius * Math.cos(theta);
            double y = centerY + radius * Math.sin(theta);
            points.add(new double[] {x, y});
        }
        return points.toArray(new double[0][0]);
    }

    /**
     * Creates a triangle obstacle from 3 vertices.
     */
    public static double[][] triangleObstacle(double[] p1, double[] p2, double[] p3) {
        return new double[][] {
                {p1[0], p1[1]},
                {p2[0], p2[1]},
                {p3[0], p3[1]}
        };
    }
}
