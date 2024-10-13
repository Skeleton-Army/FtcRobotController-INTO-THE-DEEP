package org.firstinspires.ftc.teamcode.opencv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class FastDetectSamples extends OpenCvPipeline {
    public static int[] linesArray;
    public /*final*/ OpenCvCamera webcam;
    boolean viewportPaused;

    public double h_fov = 78;
    public double v_hov = 41.4;
    public double inchLength = 1.5;
    public double focalLength = 272;
    public double imageWidth = 320;
    public double imageHeight = 240;
    public double sampleZ = -1;
    private final Telemetry telemetry;

    public FastDetectSamples(/*OpenCvCamera webcam*/ Telemetry telemetry){
        //this.webcam = webcam;
        this.telemetry = telemetry;
    }
    public Mat processFrame(Mat input) {
        Mat yellowMask = preprocessFrame(input);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Point[] coordinates = new Point[contours.size()];
        Mat contours_black = new Mat(input.size(), input.type(), new Scalar(0, 0, 0));
        Imgproc.drawContours(contours_black, contours, -1, new Scalar(255, 255, 255), -1);
        Core.bitwise_and(input, contours_black, contours_black);
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Calculate the epsilon (accuracy parameter)
            double epsilon = 0.03 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f contour_approx = new MatOfPoint2f(contour.toArray());
            Imgproc.approxPolyDP(contour2f, contour_approx, epsilon, true);
            MatOfPoint points = new MatOfPoint(contour_approx.toArray());

            Point[] vertices = points.toArray();
            telemetry.addData("Vertices", vertices.length);

            double length_pixels = calculatePixels(vertices);
            double x = calculateX(length_pixels);
            double angle = calculateHorizontalAngle(vertices[0].x);
            double y = calculateY(angle, x);

            coordinates[contours.indexOf(contour)] = new Point(x, y);

            Imgproc.putText(input, Math.round(x) + "," + Math.round(y) + "  " + Math.round(angle), vertices[0], Imgproc.FONT_HERSHEY_SIMPLEX, 0.3, new Scalar(0, 0, 0), 1);
        }
        telemetry.update();
        return input;
    }
    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2YCrCb);

        Scalar lowerYellow = new Scalar(0, 138, 0);
        Scalar upperYellow = new Scalar(255, 200, 100);


        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }
    public double calculateX(double pixelLength) {
        return  inchLength * focalLength / pixelLength;
    }
    public double calculateY(double horizontalAngle, double sampleX) {
        return  Math.tan(Math.toRadians(horizontalAngle)) * sampleX;
    }
    public double calculateHorizontalAngle(double center_x) {
        return (center_x - imageWidth / 2) * h_fov / imageWidth;
    }
    public double calculateVerticalAngle(double center_y) {
        return (center_y - imageHeight / 2) * v_hov / imageHeight;
    }
    public double calculatePixels(Point[] vertices) {
        telemetry.addData("Vertices", vertices.length);
        if (vertices.length == 6) {
            double sum = 0;
            for (int i = 0; i < vertices.length; i++) {
                Point start = vertices[i];
                Point end = vertices[(i + 1) % vertices.length];
                double length = Math.sqrt(Math.pow(start.x - end.x, 2) + Math.pow(start.y - end.y, 2));
                if (Math.abs(start.x - end.x) < 0.2 * length) {
                    sum += length;
                }
            }
            return sum / 2;
        }
        if (vertices.length == 4) {
            for (int i = 0; i < 4; i++) {
                Point start = vertices[i];
                Point end = vertices[(i + 1) % vertices.length];
                double length = Math.sqrt(Math.pow(start.x - end.x, 2) + Math.pow(start.y - end.y, 2));
                if (Math.abs(start.x - end.x) < 0.2 * length) {
                    double wanted_length = Math.tan(Math.toRadians(calculateVerticalAngle(vertices[0].y))) * length;
                    double horizontal_length = Math.sqrt(Math.pow(vertices[(i + 2) % vertices.length].x - vertices[(i + 3) % vertices.length].x, 2) + Math.pow(vertices[(i + 2) % vertices.length].y - vertices[(i + 3) % vertices.length].y, 2));
                    if (Math.abs(2.33 * wanted_length - horizontal_length) < horizontal_length * 0.2)
                        return wanted_length;
                    return horizontal_length;
                }
            }
        }
        return -1;
    }

    @Override
    public void onViewportTapped()
    {
        viewportPaused = !viewportPaused;

        if (viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }
}
