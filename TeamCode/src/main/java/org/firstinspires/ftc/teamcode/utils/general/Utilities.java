package org.firstinspires.ftc.teamcode.utils.general;

import com.acmerobotics.roadrunner.Vector2d;

public class Utilities {
    public static Vector2d rotate(Vector2d original, double angle) {
        double rx = (original.x * Math.cos(angle)) - (original.y * Math.sin(angle));
        double ry = (original.x * Math.sin(angle)) + (original.y * Math.cos(angle));

        return new Vector2d(rx, ry);
    }
}
