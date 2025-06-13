package org.firstinspires.ftc.teamcode.utils.config.cameras;

import java.util.Map;

/*
    a general class to create webcams

 */

public class Camera {
    public final String name;

    public final int width;
    public final int height;

    public final double[] cameraMatrix;
    public final double[] distCoeffs;

    public final double offsetX;
    public final double offsetY;
    public final double offsetZ;

    public final double yaw;
    public final double pitch;
    public final double roll;

    public final double fx, fy, cx, cy;
    public Map<String, Object> extra;

    public Camera(String name,
                  int width, int height,
                  double[] cameraMatrix, double[] distCoeffs,
                  double offsetX, double offsetY, double offsetZ,
                  double yaw, double pitch, double roll) {

        this.name = name;

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
        fx = cameraMatrix[0];
        fy = cameraMatrix[4];
        cx = cameraMatrix[2];
        cy = cameraMatrix[5];
    }

    @Override
    public String toString() {
        return String.format("FtcCamera[name=%s, %dx%d, offset=(%.2f, %.2f, %.2f), orientation=(yaw=%.2f, pitch=%.2f, roll=%.2f)]",
                name, width, height, offsetX, offsetY, offsetZ, yaw, pitch, roll);
    }
}
