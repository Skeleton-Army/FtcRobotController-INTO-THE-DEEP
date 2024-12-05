package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CameraConfig {
    public static float z = 13.25f; //TODO: figure out what this is
    public static float horizontalFOV = 64.4f; // Larger - more to the right
    public static float verticalFOV = 46.8f; // Larger - more backwards
    public static int halfImageWidth = 160;
    public static int halfImageHeight = 120;
    public static double offsetX = 0; //x is horizontal not IDO-style, -is when overshooting (it passes it = backwards)
    public static double offsetY = 0;

    public static float hOVERwidth = horizontalFOV / (halfImageWidth * 2);
    public static float vOVERheight = verticalFOV / (halfImageHeight * 2); //0.195
}
