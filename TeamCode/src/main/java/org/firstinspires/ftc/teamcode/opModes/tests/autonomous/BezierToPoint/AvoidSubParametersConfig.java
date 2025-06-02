package org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class AvoidSubParametersConfig {
    public static int startX = 72;
    public static int startY = 24;
    public static int endX = 20;
    public static int endY = 124;

    public static int startAngle = 90;
    public static int endAngle = 0;

    public static int width = 18;
    public static int height = 18;

    public static int widthOffset = 2;
    public static int heightOffset = 2;

    public static int stepSize = 1;
    public static int maxIterations = 25;
    public static int numSamplesPos = 50;
    public static final int fieldSize = 144;

    List<double[][]> corners = new ArrayList<>();

    public static List<double[][]> obstacles = new ArrayList<>(Arrays.asList(
            new double[][][] {new Obstacle(48, 96, 48, 48).obstacleCorners /* submersible obstacle */}
    ));


}
