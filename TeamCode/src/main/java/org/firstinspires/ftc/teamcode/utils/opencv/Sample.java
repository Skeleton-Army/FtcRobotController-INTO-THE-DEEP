package org.firstinspires.ftc.teamcode.utils.opencv;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
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
    public void calculateOrientation(Rect boundingRect, double angle2d) {
        int width = boundingRect.width;
        double constLen = Math.sqrt(Math.pow(1.5, 2) + Math.pow(3.5, 2));
        double widthToAngle = Math.toRadians(width * CameraConfig.hOVERwidth);
        double lenInches = Math.tan(widthToAngle) * (sampleY - CameraConfig.offsetY);
        //orientation = Math.asin((width * CameraConfig.hOVERwidth) / (Math.cos(horizontalAngle) * constLen)) - Math.abs(horizontalAngle) - Math.atan(1.5 / 3.5);
        double angle = Math.asin(lenInches / (Math.cos(horizontalAngle) * constLen));

        orientation = projectAndCompare(angle, angle2d);
    }

    public double projectAndCompare(double angle, double angle2d) {
        double orientationFirst = angle - Math.abs(horizontalAngle) - Math.atan(1.5 / 3.5);
        double orientationSec = 180 - angle - Math.abs(horizontalAngle) - Math.atan(1.5 / 3.5);
        double sign = horizontalAngle / Math.abs(horizontalAngle);

        if (orientationFirst < -10) {
            return Math.abs(orientationSec);
        }
        if (orientationSec > 100) {
            return Math.abs(orientationFirst);
        }

        double firstAngle = Math.atan(-projectAndGetSlope(orientationFirst));
        double secAngle = Math.atan(-projectAndGetSlope(orientationSec));

        return Math.abs(angle2d - firstAngle) < Math.abs(angle2d - secAngle) ? orientationFirst : orientationSec;
    }

    private double projectAndGetSlope(double curOrientation) {
        MatOfPoint3f objectPoints = new MatOfPoint3f();
        MatOfPoint2f imagePoints = new MatOfPoint2f();

        double sign = horizontalAngle / Math.abs(horizontalAngle);
        objectPoints.put(0,0, sampleX, 1.5 - CameraConfig.z, sampleY,
                sampleX - sign * 3.5 * Math.sin(curOrientation), 1.5 - CameraConfig.z, sampleY + 3.5 * Math.cos(curOrientation));
        Calib3d.projectPoints(objectPoints, CameraConfig.rvec, CameraConfig.tvec, CameraConfig.cameraMatrix, CameraConfig.distCoeffs, imagePoints);

        double[] start = imagePoints.get(0,0);
        double[] end = imagePoints.get(0, 1);

        return (end[1] - start[1]) / (end[0] - start[0]);
    }

    public void calculateClawTo(Rect boundingRect) {
        int width = boundingRect.width;
        double constLen = Math.sqrt(Math.pow(1.5, 2) + Math.pow(3.5, 2));
        double widthToAngle = Math.toRadians(width * CameraConfig.hOVERwidth);
        double lenInches = Math.tan(widthToAngle) * (sampleY - CameraConfig.offsetY);
        clawTo = lenInches * Math.pow(Math.cos(horizontalAngle), 2) + Math.sin(Math.abs(horizontalAngle)) *
                Math.sqrt(Math.pow(constLen, 2) - Math.pow(lenInches * Math.cos(horizontalAngle), 2));
    }
    public void calculateField() {
        double x = detectionPose.position.x + sampleY * Math.cos(detectionPose.heading.toDouble()) - sampleX * Math.sin(detectionPose.heading.toDouble());
        double y = detectionPose.position.y + sampleY * Math.sin(detectionPose.heading.toDouble()) + sampleX * Math.cos(detectionPose.heading.toDouble());
        fieldPos = new Pose2d(new Vector2d(x, y), orientation);
    }
}