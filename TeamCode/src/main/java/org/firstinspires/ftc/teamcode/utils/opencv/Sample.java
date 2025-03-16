package org.firstinspires.ftc.teamcode.utils.opencv;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

public class Sample {
    public Point lowest; // The lowest detected point of the sample in the image

    public Point center;
    private final Pose2d detectionPose; // The pose where the sample was detected
    private double sampleX, sampleY, horizontalAngle, quality;
    private double centerX, centerY;
    public double orientation;
    private Pose2d fieldPos; // Field-relative position of the sample
    public double widthInches;
    public double heightInches;

    public Sample(Point lowest, Point center, RotatedRect rect, Pose2d detectionPose) {
        this.lowest = lowest;
        this.center = center;
        this.detectionPose = detectionPose;
        calculatePosition(rect);
    }

//    public Sample(Point lowest, Pose2d detectionPose, MatOfPoint contour) {
//        this.lowest = lowest;
//        this.detectionPose = detectionPose;
//        this.contour = contour;
//        calculatePosition();
//    }

    // Getters for sample properties
    public double getSampleX() {
        return sampleX;
    }
    public double getCenterX() {
        return centerX;
    }

    public double getSampleY() {
        return sampleY;
    }
    public double getCenterY() {
        return centerY;
    }

    public double getQuality() {
        return quality;
    }

    public Pose2d getSamplePosition() {
        return fieldPos;
    }

    private Vector2d pixelToWorld(double x, double y, double height) {
        double horizontal = Math.toRadians((CameraConfig.halfImageWidth - x) * CameraConfig.hOverWidth() + CameraConfig.offsetHorizontal);
        double worldY = height / Math.tan(Math.toRadians((y - CameraConfig.halfImageHeight) * CameraConfig.vOverHeight() + CameraConfig.offsetVertical));
        double worldX = Math.tan(horizontal) * worldY;
        return new Vector2d(worldX, worldY);
    }

    /// Calculates the sample position in robot-relative coordinates
    private void calculatePosition(RotatedRect rect) {
        Vector2d lowestPos = pixelToWorld(lowest.x, lowest.y, CameraConfig.z);
        sampleY = lowestPos.y;
        sampleX = lowestPos.x;

        double angle = Math.toRadians(90 - rect.angle);
        Vector2d second = pixelToWorld(lowest.x + 50 * Math.cos(angle), lowest.y - 50 * Math.sin(angle), CameraConfig.z);
        orientation = Math.toDegrees(Math.atan((lowestPos.y - second.y) / (lowestPos.x - second.x)));

        Vector2d centerPos = pixelToWorld(center.x, center.y, CameraConfig.z - 1);
        centerY = centerPos.y;
        centerX = centerPos.x;

        // Adjust positions based on camera offsets
        sampleY += CameraConfig.offsetY;
        sampleX -= CameraConfig.offsetX;
        centerY += CameraConfig.offsetY;
        centerX -= CameraConfig.offsetX;
    }

    public void calculateArea(Rect boundingRect) {
        int width = boundingRect.width;
        double widthToAngle = Math.toRadians(width * CameraConfig.hOVERwidth);
        widthInches = Math.tan(widthToAngle) * (sampleY - CameraConfig.offsetY);

        double topYWorld = (CameraConfig.z - 1.5) / Math.tan(Math.toRadians((boundingRect.y - CameraConfig.halfImageHeight) * CameraConfig.vOverHeight() + CameraConfig.offsetVertical));
        heightInches = topYWorld - (sampleY - CameraConfig.offsetY);
    }

    // Calculates the sample position relative to the field
    public void calculateField() {
        double x = detectionPose.position.x + centerY * Math.cos(detectionPose.heading.toDouble()) - centerX * Math.sin(detectionPose.heading.toDouble());
        double y = detectionPose.position.y + centerY * Math.sin(detectionPose.heading.toDouble()) + centerX * Math.cos(detectionPose.heading.toDouble());
        fieldPos = new Pose2d(new Vector2d(x, y), orientation);
    }

    public boolean isTooBig() {
        return (widthInches * heightInches >= CameraConfig.MAX_AREA);
    }
}