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
}
