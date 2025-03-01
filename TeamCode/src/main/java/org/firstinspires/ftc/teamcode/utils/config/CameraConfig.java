package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CameraConfig {
    public static float z = 12.5f;
    public static float horizontalFOV = 65.76f; // needs tuning - TODO: check for 65.76
    public static float verticalFOV = 39.88f; // needs tuning - TODO: check for 39.88
    public static int halfImageWidth = 320;
    public static int halfImageHeight = 180;
    public static double offsetX = 6.1; //   6.25     x is horizontal not IDO-style, -is when overshooting (it passes it = backwards)
    public static double offsetY = 7.87; // 7.06
    public static double offsetHorizontal = 0;
    public static double offsetVertical = 20;

    public static double pickupSampleOffsetX = 1.77; // 1.5
    public static double pickupSampleOffsetY = 27.55; // 23

    // Apriltag settings
    public static double fx = (halfImageWidth * 2) / (2 * Math.tan(Math.toRadians(horizontalFOV)));
    public static double fy = (halfImageHeight * 2) / (2 * Math.tan(Math.toRadians(verticalFOV)));
    public static double cx = CameraConfig.halfImageWidth;
    public static double cy = CameraConfig.halfImageHeight;

    public static double yaw = 0;
    public static double pitch = -90;
    public static double roll = offsetVertical;
    public static double offsetXApriltag = 6.6;
    public static double offsetYApriltag = 8.2;
    public static double offsetZApriltag = 9.4;

    public static float hOverWidth() {
        return horizontalFOV / (halfImageWidth * 2);
    }

    public static float vOverHeight() {
        return verticalFOV / (halfImageHeight * 2);
    }
}
