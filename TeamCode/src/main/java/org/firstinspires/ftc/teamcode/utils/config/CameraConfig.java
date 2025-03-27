package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CameraConfig {
    public static double MAX_AREA = 12;
    public static float z = 12.5f;
    public static float horizontalFOV = 66.584f; // old: 65.76f
    public static float verticalFOV = 40.481f; // old: 39.88f
    public static int halfImageWidth = 320;
    public static int halfImageHeight = 180;
    public static double offsetX = 6.1; // x is horizontal not IDO-style, -is when overshooting (it passes it = backwards)
    public static double offsetY = 7.87;
    public static double offsetHorizontal = 0;
    public static double offsetVertical = 20.45;

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


    // Distortion coefficients

    // Camera matrix
    public static final double[] cameraMatrix = {
            487.30053838, 0, 323.60546032,  // fx, 0, cx
            0, 488.41988506, 168.14294789,  // 0, fy, cy
            0, 0, 1                         // 0, 0, 1
    };

    // Distortion coefficients (k1, k2, p1, p2, k3)
    public static final double[] distCoeffs = { -7.02600880e-04, 1.35749951e-01, -6.60544203e-03, 3.00740926e-04, -5.93395658e-01 };

    // Apriltag settings

    public static double fx = cameraMatrix[0];
    public static double fy = cameraMatrix[4];
    public static double cx = cameraMatrix[2];
    public static double cy = cameraMatrix[5];

    public static double yaw = 0;
    public static double pitch = -90;
    public static double roll = offsetVertical;
    public static double offsetXApriltag = 6.6;
    public static double offsetYApriltag = 8.2;
    public static double offsetZApriltag = 9.4;
    public static double vOVERheight = CameraConfig.vOverHeight();
    public static double hOVERwidth = CameraConfig.hOverWidth();

    public static float hOverWidth() {
        return horizontalFOV / (halfImageWidth * 2);
    }

    public static float vOverHeight() {
        return verticalFOV / (halfImageHeight * 2);
    }
}
