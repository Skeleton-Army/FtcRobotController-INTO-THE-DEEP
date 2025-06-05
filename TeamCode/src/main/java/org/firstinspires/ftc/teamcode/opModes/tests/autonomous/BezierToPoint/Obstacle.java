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
    public boolean isColliding(Pose pose, double robotWidth, double robotHeight, double clearance) {
        double px = pose.getVector().getXComponent();
        double py = pose.getVector().getYComponent();

        double halfRobotWidth = robotWidth / 2.0;
        double halfRobotHeight = robotHeight / 2.0;

        double left = px - halfRobotWidth - clearance;
        double right = px + halfRobotWidth + clearance;
        double top = py - halfRobotHeight - clearance;
        double bottom = py + halfRobotHeight + clearance;

        return !(right < this.x || left > this.x + this.width || bottom < this.y || top > this.y + this.height);
    }

}
