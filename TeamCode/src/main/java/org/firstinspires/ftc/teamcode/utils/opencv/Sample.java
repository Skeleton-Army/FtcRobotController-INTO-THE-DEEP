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
    private double sampleX, sampleY;
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

    public Pose2d getSamplePosition() {
        return fieldPos;
    }

    private Vector2d pixelToWorld(double x, double y, double height) {
//        double horizontal = Math.toRadians((CameraConfig.halfImageWidth - x) * CameraConfig.hOverWidth() + CameraConfig.offsetHorizontal);
//        double worldY = height / Math.tan(Math.toRadians((y - CameraConfig.halfImageHeight) * CameraConfig.vOverHeight() + CameraConfig.offsetVertical));
//        double worldX = Math.tan(horizontal) * worldY;
        //return new Vector2d(worldX, worldY);

        // Step 1. Convert pixel coordinates to normalized image coordinates.
        // x' = (u - cx) / fx, y' = (v - cy) / fy.
        double xNorm = (x - CameraConfig.cameraMatrix[2]) / CameraConfig.cameraMatrix[0];
        double yNorm = (y - CameraConfig.cameraMatrix[5]) / CameraConfig.cameraMatrix[4];

        // Step 2. Form the ray in the "level" camera coordinate system.
        // (Flip y because image coordinates increase downward.)
        // r_level = [ xNorm, -yNorm, 1 ].
        //
        // Now, rotate this ray about the x-axis by the tilt angle.
        // d = [ xNorm,
        //       -cos(tilt)*yNorm - sin(tilt),
        //       -sin(tilt)*yNorm + cos(tilt) ].
        double dX = xNorm;
        double dY = -Math.cos(Math.toRadians(CameraConfig.offsetVertical)) * yNorm - Math.sin(Math.toRadians(CameraConfig.offsetVertical));
        double dZ = -Math.sin(Math.toRadians(CameraConfig.offsetVertical)) * yNorm + Math.cos(Math.toRadians(CameraConfig.offsetVertical));

        // Step 3. Find the intersection with the ground plane (Y = 0).
        // The ray originates at the camera center: C = (camX, H, camZ).
        // A point along the ray is: P(t) = C + t * d.
        // Set the Y component to 0: H + t * dY = 0 => t = -H / dY.
        double t = -height / dY;

        // Step 4. Compute ground coordinates (in world coordinates).
        double groundX = -t * dX;
        double groundZ = t * dZ;

        return new Vector2d(groundX, groundZ);
    }

    /// Calculates the sample position in robot-relative coordinates
    private void calculatePosition(RotatedRect rect) {
        Vector2d lowestPos = pixelToWorld(lowest.x, lowest.y, CameraConfig.z);

        sampleY = lowestPos.y;
        sampleX = lowestPos.x;

        double angle = Math.toRadians(90 - rect.angle);
        Vector2d second = pixelToWorld(lowest.x + 50 * Math.cos(angle), lowest.y - 50 * Math.sin(angle), CameraConfig.z);
        orientation = Math.toDegrees(Math.atan((lowestPos.y - second.y) / (lowestPos.x - second.x)));

        Vector2d centerPos = pixelToWorld(center.x, center.y, CameraConfig.z - 0.75);
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

    public boolean isTooSmall() {
        return (widthInches * heightInches <= CameraConfig.MIN_AREA);
    }
}