package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

public class Obstacle {
    public double x, y, width, height;

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
}
