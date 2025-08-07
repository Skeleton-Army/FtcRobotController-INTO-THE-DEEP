package org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;
import java.util.List;

@Config
public class AvoidSubParametersConfig {
    public static int startX = 22;
    public static int startY = 22;
    public static int endX = 96;
    public static int endY = 24;

    public static double startAngle = 0;
    public static double endAngle = 0;
    public static final double width = 18.0;  // robot width
    public static final double height = 18.0; // robot height
    public static double widthOffset = 0;
    public static double heightOffset = 0;
    public static final double fieldSize = 144.0;


    public static final List<double[][]> obstacles = Arrays.asList(new double[][][] {
            {
                    {48, 48},
                    {96, 48},
                    {96, 96},
                    {48, 96}
            },

            // add any obstacles according the current season's
            /*{
                    {24,72},
                    {48,72},
                    {48,120},
                    {24,120},
            },
            {
                    {24 + 24 * 3,72 + 24 * 3},
                    {48 + 24 * 3,72 + 24 * 3},
                    {48 + 24 * 3,120 + 24 * 3},
                    {24 + 24 * 3,120 + 24 * 3},
            },
            {
                    {72,24},
                    {96,24},
                    {96,48},
                    {72,48},
            }*/
    });


}