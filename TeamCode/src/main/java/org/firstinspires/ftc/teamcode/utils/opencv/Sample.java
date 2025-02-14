package org.firstinspires.ftc.teamcode.utils.opencv;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;

public class Sample {
    private final Pose2d detectionPose;
    private double sampleX, sampleY, horizontalAngle, quality, orientation, clawTo;
    private Pose2d fieldPos;
    public Point lowest;
    public Sample(Point lowest, Pose2d detectionPose) {
        this.lowest = lowest;
        this.detectionPose = detectionPose;
        calculatePosition();
    }

    public double getSampleX() {
        return sampleX;
    }

    public double getSampleY() {
        return sampleY;
    }

    public double getClawTo() {
        return clawTo;
    }
    public double getQuality() {
        return quality;
    }
    public Pose2d getSamplePosition() {
        return fieldPos;
    }
    private void calculatePosition() {
        horizontalAngle = Math.toRadians((CameraConfig.halfImageWidth - lowest.x) * CameraConfig.hOVERwidth + CameraConfig.offsetHorizontal);
        sampleY = CameraConfig.z / Math.tan(Math.toRadians((lowest.y - CameraConfig.halfImageHeight) * CameraConfig.vOVERheight + CameraConfig.offsetVertical));
        sampleX = Math.tan(horizontalAngle) * sampleY;
        sampleY += CameraConfig.offsetY;
        sampleX -= CameraConfig.offsetX;
    }

    public void findQuality(MatOfPoint contour) {
        int width = contour.width();
        double bestCase = Math.toDegrees(Math.atan((1.5 + Math.abs(3.5 * Math.sin(horizontalAngle))) / sampleY - CameraConfig.offsetY) / CameraConfig.hOVERwidth);
        quality = bestCase / width;
    }
    public void calculateOrientation(Rect boundingRect) {
        int width = boundingRect.width;
        double constLen = Math.sqrt(Math.pow(1.5, 2) + Math.pow(3.5, 2));
        double widthToAngle = Math.toRadians(width * CameraConfig.hOVERwidth);
        double lenInches = Math.tan(widthToAngle) * (sampleY - CameraConfig.offsetY);
        //orientation = Math.asin((width * CameraConfig.hOVERwidth) / (Math.cos(horizontalAngle) * constLen)) - Math.abs(horizontalAngle) - Math.atan(1.5 / 3.5);
        double angle = Math.asin(lenInches / (Math.cos(horizontalAngle) * constLen));
        orientation = angle - Math.atan(1.5 / 3.5) - Math.abs(horizontalAngle);
        //if (orientation >= Math.atan(1.5 / 3.5)) {
            //orientation = Math.PI - angle + Math.atan(1.5 / 3.5) - Math.abs(horizontalAngle);
        //}
    }

    public void calculateClawTo(Rect boundingRect) {
        int width = boundingRect.width;
        double constLen = Math.sqrt(Math.pow(1.5, 2) + Math.pow(3.5, 2));
        double widthToAngle = Math.toRadians(width * CameraConfig.hOVERwidth);
        double lenInches = Math.tan(widthToAngle) * (sampleY - CameraConfig.offsetY);
        clawTo = lenInches * Math.pow(Math.cos(horizontalAngle), 2);
        double sqrt = Math.sin(horizontalAngle) * Math.sqrt(Math.pow(constLen, 2) - Math.pow(lenInches * Math.cos(horizontalAngle), 2));
        if (clawTo - sqrt < 1.5){
            clawTo += sqrt;
        }
        else {
            clawTo -= sqrt;
        }
    }
    public void calculateField() {
        double x = detectionPose.position.x + sampleY * Math.cos(detectionPose.heading.toDouble()) - sampleX * Math.sin(detectionPose.heading.toDouble());
        double y = detectionPose.position.y + sampleY * Math.sin(detectionPose.heading.toDouble()) + sampleX * Math.cos(detectionPose.heading.toDouble());
        fieldPos = new Pose2d(new Vector2d(x, y), orientation);
    }
}