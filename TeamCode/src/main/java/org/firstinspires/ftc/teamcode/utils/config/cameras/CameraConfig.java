package org.firstinspires.ftc.teamcode.utils.config.cameras;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CameraConfig {
    public static double MAX_AREA = 12;
    public static double MIN_AREA = 6;
    public static float horizontalFOV = 66.584f; // old: 65.76f
    public static float verticalFOV = 40.481f; // old: 39.88f
    public static double offsetHorizontal = 0;
    public static double offsetVertical = 18;

    public static double pickupSampleOffsetX = 1.5;
    public static double pickupSampleOffsetY = 26;

    public static double pickupInterval = 0.6;
    public static double pickupIntervalDivision = 1.3;
    public static double pickupMinInterval = 0.3;
    public static double pickupTimeout = 1.8;

    public static double pickupSpeed = 50;

    public static int pixelThreshRadius = 20;
    public static int pixelOptimalCenterX = 165;
    public static int pixelOptimalCenterY = 260;

    public static double wiggleDistance = 1.5;
    public static double wiggleBackDistance = 0.5;
    public static double vOVERheight = CameraConfig.vOverHeight();
    public static double hOVERwidth = CameraConfig.hOverWidth();

    public static long exposure = 2;
    public static long focus = 13;
    public static float hOverWidth() {
        return horizontalFOV / (CamerasManager.getByName("webcam 1").width * 2);
    }

    public static float vOverHeight() {
        return verticalFOV / (CamerasManager.getByName("webcam 1").height * 2);
    }
}
