package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CameraConfig {
    public static float z = 13.125f;
    public static float horizontalFOV = 63.62f; // needs tuning
    public static float verticalFOV = 35.78f; // needs tuning
    public static int halfImageWidth = 320;
    public static int halfImageHeight = 180;
    public static double offsetX = 0; //x is horizontal not IDO-style, -is when overshooting (it passes it = backwards)
    public static double offsetY = 0;
    public static double offsetHorizontal = 0;
    public static double offsetVertical = 0;
    public static float hOVERwidth = horizontalFOV / (halfImageWidth * 2);
    public static float vOVERheight = verticalFOV / (halfImageHeight * 2); //0.195

    // apriltag settings

    public static double fx = 3.6/(2*Math.tan((CameraConfig.horizontalFOV) * Math.PI)/180)/(3.6/1280);
    public static double fy = 2.7/(2*Math.tan((CameraConfig.verticalFOV) * Math.PI)/180)/(2.7/720);
    public static double cx = CameraConfig.halfImageWidth * 2;
    public static double cy = CameraConfig.halfImageHeight * 2;

    public static double yaw = 0;
    public static double pitch = -90;
    public static double roll = 0;
    public static double offsetXApriltag = 6.6;
    public static double offsetYApriltag = 8.2;
    public static double offsetZApriltag = 9.4;


}
