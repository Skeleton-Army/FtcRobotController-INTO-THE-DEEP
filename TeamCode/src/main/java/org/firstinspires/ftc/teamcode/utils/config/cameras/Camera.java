package org.firstinspires.ftc.teamcode.utils.config.cameras;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.ApriltagOfCamera;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;

public class Camera {

    // resolutions
    public  final int width;
    public final int height;

    // calibrations values
    double[] cameraMatrix;
    double[] distCoeffs;

    // cameras positions for apriltag and more
    // all offsets from the center of the robot
    public final double offsetX;
    public final double offsetY;
    public final double offsetZ;

    public final double yaw;
    public final double pitch; // also know as the vertical offset
    public final double roll;

    public Camera(int width,int height,double[] cameraMatrix, double[] distCoeffs, double offsetX, double offsetY, double offsetZ, double yaw, double pitch, double roll) {
        this.width = width;
        this.height = height;
        this.cameraMatrix = cameraMatrix;
        this.distCoeffs = distCoeffs;
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        this.offsetZ = offsetZ;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }

    public ApriltagOfCamera createAprilTag() {
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                yaw, pitch, roll, 0);
        Position cameraPosition = new Position(DistanceUnit.INCH,
                offsetX, offsetY, offsetZ, 0);

        ApriltagOfCamera apriltag = new ApriltagOfCamera(cameraMatrix[0], cameraMatrix[4],cameraMatrix[2],cameraMatrix[5], cameraPosition, cameraOrientation);
        return apriltag;
    }

}
