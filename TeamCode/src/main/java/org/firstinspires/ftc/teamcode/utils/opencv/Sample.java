package org.firstinspires.ftc.teamcode.utils.opencv;

import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.Point;
public class Sample {
    private double sampleX, sampleY, orientation, distance, horizontalAngle;
    public Point reference;
    private Point[] vertices;
    public Sample(Point[] vertices) {
        this.vertices = vertices;
        calculatePosition();
        //calculateOrientation();

        distance = Math.sqrt(Math.pow(sampleX, 2) + Math.pow(sampleY, 2));
    }
    public double getSampleX() {return sampleX;}
    public double getSampleY() {return sampleY;}
    public double getOrientation() {return orientation;}
    public double getDistance() {return distance;}
    public double getHorizontalAngle() {return horizontalAngle;}

    private void calculatePosition() {
        reference = vertices[0];
        for (Point vertex : vertices) {
            if (vertex.y > reference.y) {
                reference = vertex;
            }
        }
        horizontalAngle = Math.toRadians((CameraConfig.halfImageWidth - reference.x) * CameraConfig.hOVERwidth + CameraConfig.offsetHorizontal);
        sampleY = CameraConfig.z / Math.tan(Math.toRadians((reference.y - CameraConfig.halfImageHeight) * CameraConfig.vOVERheight));
        sampleX = Math.tan(horizontalAngle) * sampleY;
        sampleY += CameraConfig.offsetY;
        sampleX -= CameraConfig.offsetX;
    }
    private void calculateOrientation() {
        int index = 0;
        for (int i = 0; i < vertices.length; i++) {
            if (vertices[i].y < vertices[index].y) {
                index = i;
            }
        }
        Point point1 = vertices[(index - 1 + vertices.length) % vertices.length];
        Point point2 = vertices[index];
        Point point3 = vertices[(index + 1) % vertices.length];
        double verticalAngle = (point2.y - CameraConfig.halfImageHeight) * CameraConfig.vOVERheight + CameraConfig.offsetVertical;
        double wanted_length_squared = Math.pow(point3.y - point2.y, 2) + Math.pow(point2.x - point3.x, 2);
        double other_length_squared = Math.pow(point1.y - point2.y, 2) + Math.pow(point2.x - point1.x, 2);
        if (other_length_squared < wanted_length_squared)
            point1 = point3;
        orientation =  Math.atan((point1.y - point2.y) / ((point2.x - point1.x) * Math.tan(Math.toRadians(verticalAngle))));
    }
}