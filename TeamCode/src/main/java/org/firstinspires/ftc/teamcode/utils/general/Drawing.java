package org.firstinspires.ftc.teamcode.utils.general;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.localization.Pose;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, Pose t, String color) {
        final double ROBOT_RADIUS = 9;

        Pose2d tRR = new Pose2d(t.getX(), t.getY(), t.getHeading());
        c.setStrokeWidth(1);
        c.setStroke(color);
        c.strokeCircle(tRR.position.x, tRR.position.y, ROBOT_RADIUS);

        Vector2d vec1 = new Vector2d(tRR.position.x, tRR.position.y);
        Vector2d halfv = tRR.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = vec1.plus(halfv);

        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}