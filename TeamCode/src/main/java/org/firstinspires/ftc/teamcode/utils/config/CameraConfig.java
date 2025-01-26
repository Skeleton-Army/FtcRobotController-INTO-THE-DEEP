package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CameraConfig {
    public static float z = 13.125f;

    public static float horizontalFOV = 65.64f;
    public static float verticalFOV = 39.88f;
    public static int halfImageWidth = 320;
    public static int halfImageHeight = 180;

    public static double offsetX = 0;
    public static double offsetY = 0;
    public static double offsetHorizontal = 0; // 0 = Forward of the robot
    public static double offsetVertical = 8.5; // 0 = Parallel to the ground

    public static float hOVERwidth = horizontalFOV / (halfImageWidth * 2);
    public static float vOVERheight = verticalFOV / (halfImageHeight * 2);

    public static double fx = (CameraConfig.halfImageWidth * 2) / (2*Math.tan(Math.toRadians(horizontalFOV)));
    public static double fy = (CameraConfig.halfImageHeight * 2) / (2*Math.tan(Math.toRadians(verticalFOV)));
    public static double cx = CameraConfig.halfImageWidth;
    public static double cy = CameraConfig.halfImageHeight;
}
