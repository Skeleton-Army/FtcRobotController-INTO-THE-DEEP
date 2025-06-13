package org.firstinspires.ftc.teamcode.utils.opencv;

import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.utils.config.cameras.Camera;
import org.firstinspires.ftc.teamcode.utils.config.cameras.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.config.cameras.CamerasManager;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

public class Sample {
    public Point lowest; // The lowest detected point of the sample in the image

    public Point center;
    private final Pose detectionPose; // The pose where the sample was detected
    private double sampleX, sampleY, horizontalAngle, quality;
    private double centerX, centerY;
    public double orientation;
    private Pose fieldPos; // Field-relative position of the sample
    public double widthInches;
    public double heightInches;

    Camera camera = CamerasManager.getByName("Webcam 1");

    public Sample(Point lowest, Point center, RotatedRect rect, Pose detectionPose) {
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

    public Pose getSamplePosition() {
        return fieldPos;
    }

    private Pose pixelToWorld(double x, double y, double height) {
//        double horizontal = Math.toRadians((CameraConfig.halfImageWidth - x) * CameraConfig.hOverWidth() + CameraConfig.offsetHorizontal);
//        double worldY = height / Math.tan(Math.toRadians((y - CameraConfig.halfImageHeight) * CameraConfig.vOverHeight() + CameraConfig.offsetVertical));
//        double worldX = Math.tan(horizontal) * worldY;
        //return new Vector2d(worldX, worldY);

        // Step 1. Convert pixel coordinates to normalized image coordinates.
        // x' = (u - cx) / fx, y' = (v - cy) / fy.
        double xNorm = (x - camera.cameraMatrix[2]) / camera.cameraMatrix[0];
        double yNorm = (y - camera.cameraMatrix[5]) / camera.cameraMatrix[4];

        // Step 2. Form the ray in the "level" camera coordinate system.
        // (Flip y because image coordinates increase downward.)
        // r_level = [ xNorm, -yNorm, 1 ].
        //
        // Now, rotate this ray about the x-axis by the tilt angle.
        // d = [ xNorm,
        //       -cos(tilt)*yNorm - sin(tilt),
        //       -sin(tilt)*yNorm + cos(tilt) ].
        double dX = xNorm;
        double dY = -Math.cos(Math.toRadians(camera.pitch)) * yNorm - Math.sin(Math.toRadians(camera.pitch));
        double dZ = -Math.sin(Math.toRadians(camera.pitch)) * yNorm + Math.cos(Math.toRadians(camera.pitch));

        // Step 3. Find the intersection with the ground plane (Y = 0).
        // The ray originates at the camera center: C = (camX, H, camZ).
        // A point along the ray is: P(t) = C + t * d.
        // Set the Y component to 0: H + t * dY = 0 => t = -H / dY.
        double t = -height / dY;

        // Step 4. Compute ground coordinates (in world coordinates).
        double groundX = -t * dX;
        double groundZ = t * dZ;

        return new Pose(groundX, groundZ);
    }

    /// Calculates the sample position in robot-relative coordinates
    private void calculatePosition(RotatedRect rect) {
        Pose lowestPos = pixelToWorld(lowest.x, lowest.y, camera.offsetZ);

        sampleY = lowestPos.getY();
        sampleX = lowestPos.getX();

        double angle = Math.toRadians(90 - rect.angle);
        Pose second = pixelToWorld(lowest.x + 50 * Math.cos(angle), lowest.y - 50 * Math.sin(angle), CamerasManager.getByName("Webcam 1").offsetZ);
        orientation = Math.toDegrees(Math.atan((lowestPos.getY() - second.getY()) / (lowestPos.getX() - second.getX())));

        Pose centerPos = pixelToWorld(center.x, center.y, CamerasManager.getByName("Webcam 1").offsetZ - 0.75);
        centerY = centerPos.getY();
        centerX = centerPos.getX();

        // Adjust positions based on camera offsets
        sampleY += camera.offsetY;
        sampleX -= camera.offsetX;
        centerY += camera.offsetY;
        centerX -= camera.offsetX;
    }

    public void calculateArea(Rect boundingRect) {
        int width = boundingRect.width;
        double widthToAngle = Math.toRadians(width * CameraConfig.hOVERwidth);
        widthInches = Math.tan(widthToAngle) * (sampleY - camera.offsetY);

        double topYWorld = (camera.offsetZ - 1.5) / Math.tan(Math.toRadians((boundingRect.y - (double) camera.height / 2) * CameraConfig.vOverHeight() + CameraConfig.offsetVertical));
        heightInches = topYWorld - (sampleY - camera.offsetY);
    }

    // Calculates the sample position relative to the field
    public void calculateField() {
        double x = detectionPose.getX() + centerY * Math.cos(detectionPose.getHeading()) - centerX * Math.sin(detectionPose.getHeading());
        double y = detectionPose.getY() + centerY * Math.sin(detectionPose.getHeading()) + centerX * Math.cos(detectionPose.getHeading());
        fieldPos = new Pose(x, y, orientation);
    }

    public boolean isTooBig() {
        return (widthInches * heightInches >= CameraConfig.MAX_AREA);
    }

    public boolean isTooSmall() {
        return (widthInches * heightInches <= CameraConfig.MIN_AREA);
    }
}