package org.firstinspires.ftc.teamcode.utils.opencv;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.Point;
public class Sample {
    private final Pose2d detectionPose;
    private double sampleX, sampleY;
    private Vector2d fieldPos;
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
    public Vector2d getSamplePosition() {
        return fieldPos;
    }
    private void calculatePosition() {
        double horizontalAngle = Math.toRadians((CameraConfig.halfImageWidth - lowest.x) * CameraConfig.hOVERwidth + CameraConfig.offsetHorizontal);
        double sampleY = CameraConfig.z / Math.tan(Math.toRadians((lowest.y - CameraConfig.halfImageHeight) * CameraConfig.vOVERheight + CameraConfig.offsetVertical));
        double sampleX = Math.tan(horizontalAngle) * sampleY;
        sampleY += CameraConfig.offsetY;
        sampleX -= CameraConfig.offsetX;
        double x = detectionPose.position.x + sampleY * Math.cos(detectionPose.heading.toDouble()) - sampleX * Math.sin(detectionPose.heading.toDouble());
        double y = detectionPose.position.y + sampleY * Math.sin(detectionPose.heading.toDouble()) + sampleX * Math.cos(detectionPose.heading.toDouble());
        fieldPos = new Vector2d(x, y);
    }
}