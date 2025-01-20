package org.firstinspires.ftc.teamcode.utils.opencv;

import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.Point;
public class Sample {
    private double sampleX, sampleY, distance, horizontalAngle;
    public Point lowest;
    public Sample(Point lowest) {
        this.lowest = lowest;
        calculatePosition();
        distance = Math.sqrt(Math.pow(sampleX, 2) + Math.pow(sampleY, 2));
    }
    public double getSampleX() {return sampleX;}
    public double getSampleY() {return sampleY;}
    public double getDistance() {return distance;}
    public double getHorizontalAngle() {return horizontalAngle;}

    private void calculatePosition() {
        horizontalAngle = Math.toRadians((CameraConfig.halfImageWidth - lowest.x) * CameraConfig.hOVERwidth + CameraConfig.offsetHorizontal);
        sampleY = CameraConfig.z / Math.tan(Math.toRadians((lowest.y - CameraConfig.halfImageHeight) * CameraConfig.vOVERheight + CameraConfig.offsetVertical));
        sampleX = Math.tan(horizontalAngle) * sampleY;
        sampleY += CameraConfig.offsetY;
        sampleX -= CameraConfig.offsetX;
    }
}