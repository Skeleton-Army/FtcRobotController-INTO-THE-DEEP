package org.firstinspires.ftc.teamcode.utils.opencv;

import android.graphics.Camera;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

public class Sample {
    public Point lowest; // The lowest detected point of the sample in the image

    private final Pose2d detectionPose; // The pose where the sample was detected
    private double sampleX, sampleY, horizontalAngle, quality;
    public double orientation;
    private Pose2d fieldPos; // Field-relative position of the sample
    private MatOfPoint contour;
    public double widthInches;
    public double heightInches;

    public Sample(Point lowest, Pose2d detectionPose, RotatedRect minRect) {
        this.lowest = lowest;
        this.detectionPose = detectionPose;
        calculatePosition(minRect);
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

    public double getSampleY() {
        return sampleY;
    }

    public double getQuality() {
        return quality;
    }

    public Pose2d getSamplePosition() {
        return fieldPos;
    }

    public MatOfPoint getContour() {
        return this.contour;
    }

    private Vector2d pixelToWorld(double x, double y) {
        double horizontal = Math.toRadians((CameraConfig.halfImageWidth - x) * CameraConfig.hOverWidth() + CameraConfig.offsetHorizontal);
        double worldY = CameraConfig.z / Math.tan(Math.toRadians((y - CameraConfig.halfImageHeight) * CameraConfig.vOverHeight() + CameraConfig.offsetVertical));
        double worldX = Math.tan(horizontal) * worldY;
        return new Vector2d(worldX, worldY);
    }

    /// Calculates the sample position in robot-relative coordinates
    private void calculatePosition(RotatedRect minRect) {
        Vector2d lowestPos = pixelToWorld(lowest.x, lowest.y);
        sampleY = lowestPos.y;
        sampleX = lowestPos.x;

        double rectAngle = Math.toRadians(minRect.angle);
        Vector2d second = pixelToWorld(lowest.x + 40 * Math.cos(rectAngle), lowest.y - 40 * Math.sin(rectAngle));
        orientation = Math.toDegrees(Math.atan((lowestPos.y - second.y) / (lowestPos.x - second.x)));
        // Adjust positions based on camera offsets
        sampleY += CameraConfig.offsetY;
        sampleX -= CameraConfig.offsetX;
    }

    // Estimates the orientation of the detected sample based on its contour

    public void calculateArea(Rect boundingRect) {
        int width = boundingRect.width;
        double widthToAngle = Math.toRadians(width * CameraConfig.hOVERwidth);
        widthInches = Math.tan(widthToAngle) * (sampleY - CameraConfig.offsetY);

        int height = boundingRect.height;
        double heightToAngle = Math.toRadians(height * CameraConfig.vOVERheight);
        heightInches = Math.tan(heightToAngle) * (sampleY - CameraConfig.offsetY);
    }

    // Calculates the sample position relative to the field
    public void calculateField() {
        double x = detectionPose.position.x + sampleY * Math.cos(detectionPose.heading.toDouble()) - sampleX * Math.sin(detectionPose.heading.toDouble());
        double y = detectionPose.position.y + sampleY * Math.sin(detectionPose.heading.toDouble()) + sampleX * Math.cos(detectionPose.heading.toDouble());
        fieldPos = new Pose2d(new Vector2d(x, y), orientation);
    }

    public boolean isTooBig() {
        return (widthInches * heightInches >= CameraConfig.MAX_AREA);
    }
}