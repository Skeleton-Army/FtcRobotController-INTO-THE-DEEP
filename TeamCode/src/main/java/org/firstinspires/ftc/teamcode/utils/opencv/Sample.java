package org.firstinspires.ftc.teamcode.utils.opencv;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
public class Sample {
    public Point lowest; // The lowest detected point of the sample in the image

    private final Pose2d detectionPose; // The pose where the sample was detected
    private double sampleX, sampleY, horizontalAngle, quality, orientation;
    private Pose2d fieldPos; // Field-relative position of the sample

    public Sample(Point lowest, Pose2d detectionPose) {
        this.lowest = lowest;
        this.detectionPose = detectionPose;
        calculatePosition();
    }

    // Getters for sample properties
    public double getSampleX() {
        return sampleX;
    }

    public double getSampleY() {
        return sampleY;
    }

    public double getQuality() {
        return quality;
    }

    public Pose2d getSamplePosition() {
        return fieldPos;
    }

    /// Calculates the sample position in robot-relative coordinates
    private void calculatePosition() {
        horizontalAngle = Math.toRadians((CameraConfig.halfImageWidth - lowest.x) * CameraConfig.hOverWidth() + CameraConfig.offsetHorizontal);
        sampleY = CameraConfig.z / Math.tan(Math.toRadians((lowest.y - CameraConfig.halfImageHeight) * CameraConfig.vOverHeight() + CameraConfig.offsetVertical));
        sampleX = Math.tan(horizontalAngle) * sampleY;

        // Adjust positions based on camera offsets
        sampleY += CameraConfig.offsetY;
        sampleX -= CameraConfig.offsetX;
    }

    /// Determines the quality of the sample based on contour width
    public void findQuality(MatOfPoint contour) {
        int width = contour.width();
        double bestCase = Math.toDegrees(Math.atan((1.5 + Math.abs(2.5 * Math.sin(horizontalAngle))) / sampleY - CameraConfig.offsetY) / CameraConfig.hOverWidth());
        quality = bestCase / width;
    }

    // Estimates the orientation of the detected sample based on its contour
    public void calculateOrientation(MatOfPoint contour) {
        int width = contour.width();
        double constLen = Math.sqrt(Math.pow(1.5, 2) + Math.pow(2.5, 2)); // Predefined length ratio
        double widthToAngle = Math.toRadians(width * CameraConfig.hOverWidth()); // Convert width to an angular measurement
        double lenInches = Math.tan(widthToAngle) * (sampleY - CameraConfig.offsetY); // Estimate physical length in inches

        //orientation = Math.asin((width * CameraConfig.hOVERwidth) / (Math.cos(horizontalAngle) * constLen)) - Math.abs(horizontalAngle) - Math.atan(1.5 / 2.5);
        orientation = Math.asin(lenInches * constLen / Math.cos(horizontalAngle)) - Math.atan(1.5 / 2.5) - horizontalAngle;
    }

    // Calculates the sample position relative to the field
    public void calculateField() {
        double x = detectionPose.position.x + sampleY * Math.cos(detectionPose.heading.toDouble()) - sampleX * Math.sin(detectionPose.heading.toDouble());
        double y = detectionPose.position.y + sampleY * Math.sin(detectionPose.heading.toDouble()) + sampleX * Math.cos(detectionPose.heading.toDouble());
        fieldPos = new Pose2d(new Vector2d(x, y), orientation);
    }
}