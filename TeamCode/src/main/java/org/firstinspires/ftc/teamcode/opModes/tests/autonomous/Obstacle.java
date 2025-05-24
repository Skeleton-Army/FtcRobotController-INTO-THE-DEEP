package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

public  class Obstacle {
    public final double x, y, width, height;

    public Obstacle(double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    public boolean isColliding(Pose pose, double robotWidth, double robotHeight) {
        Vector point = pose.getVector();
        double px = point.getXComponent();
        double py = point.getYComponent();

        double left = px - robotWidth / 2.0;
        double right = px + robotWidth / 2.0;
        double top = py - robotHeight / 2.0;
        double bottom = py + robotHeight / 2.0;

        return (right > x && left < x + width && bottom > y && top < y + height);
    }
}

