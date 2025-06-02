package org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint;

import com.pedropathing.localization.Pose;

/**
 * Represents a rectangular obstacle in the field.
 * x, y refer to the top-left corner of the rectangle.
 */
public class Obstacle {
    public final double x, y, width, height;

    public double[][] obstacleCorners;

    public Obstacle(double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;

        obstacleCorners = new double[][] {
                {x,y},
                {x + width, y},
                {x, y + height},
                {x + width, y + height}
        };
    }

    /**
     * Checks if a given Pose (with heading and robot dimensions) collides with this obstacle.
     * Accounts for robot rotation by checking each corner of its bounding box.
     */
    public boolean isColliding(Pose pose, double robotWidth, double robotHeight) {
        double cx = pose.getVector().getXComponent();
        double cy = pose.getVector().getYComponent();
        double heading = pose.getHeading();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double halfW = robotWidth / 2.0;
        double halfH = robotHeight / 2.0;

        double[][] corners = new double[][] {
                {-halfW, -halfH},
                { halfW, -halfH},
                { halfW,  halfH},
                {-halfW,  halfH}
        };

        for (double[] corner : corners) {
            double localX = corner[0];
            double localY = corner[1];

            double rotatedX = localX * cos - localY * sin + cx;
            double rotatedY = localX * sin + localY * cos + cy;

            if (rotatedX >= x && rotatedX <= x + width && rotatedY >= y && rotatedY <= y + height) {
                return true;
            }
        }

        return false;
    }
}
