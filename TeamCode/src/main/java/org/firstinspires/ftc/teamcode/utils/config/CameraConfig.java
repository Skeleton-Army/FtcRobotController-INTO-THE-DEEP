package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.MatOfDouble;

@Config
public class CameraConfig {
    public static float z = 12.5f;
    public static float horizontalFOV = 65.76f; // needs tuning - TODO: check for 65.76
    public static float verticalFOV = 39.88f; // needs tuning - TODO: check for 39.88
    public static int halfImageWidth = 320;
    public static int halfImageHeight = 180;
    public static double offsetX = 6.1; // x is horizontal not IDO-style, -is when overshooting (it passes it = backwards)
    public static double offsetY = 7.87;
    public static double offsetHorizontal = 4.3;
    public static double offsetVertical = 22.05;

    public static double pickupSampleOffsetX = 1.25;
    public static double pickupSampleOffsetY = 24;

    public static double pickupInterval = 0.5;
    public static double pickupIntervalDivision = 1.4;
    public static double pickupTimeout = 3;

    public static int pixelThreshMinX = 108;
    public static int pixelThreshMaxX = 135;
    public static int pixelThreshMinY = 305;
    public static int pixelThreshMaxY = 335;


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
