package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

@Config
public class CameraConfig {
    public static float z = 12.5f;
    public static float horizontalFOV = 63.62f; // needs tuning
    public static float verticalFOV = 35.78f; // needs tuning
    public static int halfImageWidth = 320;
    public static int halfImageHeight = 180;
    public static double offsetX = 6.25; //x is horizontal not IDO-style, -is when overshooting (it passes it = backwards)
    public static double offsetY = 7.06;
    public static double offsetHorizontal = 0;
    public static double offsetVertical = 22.6;
    public static float hOVERwidth = horizontalFOV / (halfImageWidth * 2);
    public static float vOVERheight = verticalFOV / (halfImageHeight * 2); //0.195

    // apriltag settings

    public static double fx = (halfImageWidth * 2) / (2 * Math.tan(Math.toRadians(horizontalFOV)));
    public static double fy = (halfImageHeight * 2) / (2 * Math.tan(Math.toRadians(verticalFOV)));
    public static double cx = CameraConfig.halfImageWidth;
    public static double cy = CameraConfig.halfImageHeight;

    // =========================================================================
    // TODO: Calibrate the following variables with chessboard ASAP
    // https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html
    public static Mat cameraMatrix;        // 3x3 camera intrinsic matrix
    public static MatOfDouble distCoeffs = null;  // Distortion coefficients :TODO: This.
    public static Mat rvec;      // Camera rotation vector
    public static Mat tvec = new Mat(3, 1, CvType.CV_64FC1);
    // =========================================================================

    public static double yaw = 0;
    public static double pitch = -90;
    public static double roll = offsetVertical;
    public static double offsetXApriltag = 6.6;
    public static double offsetYApriltag = 8.2;
    public static double offsetZApriltag = 9.4;

    public static double pickupSampleOffsetX = 0.5;
    public static double pickupSampleOffsetY = 22.5;

}
