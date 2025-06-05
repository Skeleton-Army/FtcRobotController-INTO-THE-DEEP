package org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint;

import com.acmerobotics.dashboard.config.Config;

import java.lang.reflect.Array;
import java.util.List;

@Config
public class AvoidSubParametersConfig {
    public static int startX = 96;
    public static int startY = 24;
    public static int endX = 96;
    public static int endY = 120;

    public static double startAngle = 90;
    public static double endAngle = 0;
    public static double influenceRadius = 30.0;
    public static int numSamplesPos = 150;

    public static double stepSize = 10; // how far to push away each iteration
    public static int maxIterations = 300;
    public static double minimalClearance = 2.0; // inches away from obstacles
    public static final double width = 18.0;  // robot width
    public static final double height = 18.0; // robot height
    public static double widthOffset = 2;
    public static double heightOffset = 2;
    public static final double fieldSize = 144.0;

    public static final List<double[][]> obstacles =

}