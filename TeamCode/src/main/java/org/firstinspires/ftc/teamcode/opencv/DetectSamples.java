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

public class DetectSamples extends OpenCvPipeline {

    public OpenCvCamera webcam;
    boolean viewportPaused; //Do we really need this?
    private final Telemetry telemetry;

    //yellow:
    private final Scalar lowerBoundMask = new Scalar(0, 138, 0);
    private final Scalar upperBoundMask = new Scalar(255, 200, 100);


    public DetectSamples(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public Mat processFrame(Mat input) {
        Mat Mask = mask(input);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(Mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        //is this solely for us to see that it works, or does it have any computational value? Iddo: i think we wanted to use it for edge detection but it's not needed anymore
        Mat contours_black = new Mat(input.size(), input.type(), new Scalar(0, 0, 0));
        Imgproc.drawContours(contours_black, contours, -1, new Scalar(255, 255, 255), -1);
        Core.bitwise_and(input, contours_black, contours_black);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            double epsilon = 0.025 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f contour_approx = new MatOfPoint2f(contour.toArray());
            Imgproc.approxPolyDP(contour2f, contour_approx, epsilon, true);

            MatOfPoint points = new MatOfPoint(contour_approx.toArray()); //why are we doing this only to instantly make this an array again?
            Point[] vertices = points.toArray();

            double x = Camera.inchXfocal / calculatePixels(vertices);
            double x_new_function = calculateXWithZ(vertices);
            double angle = (vertices[0].x - Camera.halfImageWidth) * Camera.hOVERwidth;
            double y = Math.tan(Math.toRadians(angle)) * x;
            double y_new_function = Math.tan(Math.toRadians(angle)) * x_new_function;

            //This is just so we can check if it works, right? Iddo: yes
            Imgproc.putText(input, Math.round(x) + "," + Math.round(y) + "  " + Math.round(angle), vertices[0], Imgproc.FONT_HERSHEY_SIMPLEX, 0.3, new Scalar(0, 0, 0), 1);
        }
        telemetry.update();
        return input;
    }



    private Mat mask(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2YCrCb);

        Mat Mask = new Mat();
        Core.inRange(hsvFrame, lowerBoundMask, upperBoundMask, Mask);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(Mask, Mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(Mask, Mask, Imgproc.MORPH_CLOSE, kernel);

        return Mask;
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

        else if (vertices.length == 4) {
            for (int i = 0; i < 4; i++) {
                Point start = vertices[i];
                Point end = vertices[(i + 1) % vertices.length];
                double length = Math.sqrt(Math.pow(start.x - end.x, 2) + Math.pow(start.y - end.y, 2));
                if (Math.abs(start.x - end.x) < 0.2 * length) {
                    double wanted_length = length / (1 + Math.tan(Math.toRadians((vertices[0].y - Camera.halfImageHeight) * Camera.vOVERheight))); //I saw this wasn't up to date with what we did on 15.10, recovered from what I remembered
                    double horizontal_length = Math.sqrt(Math.pow(vertices[(i + 2) % vertices.length].x - vertices[(i + 3) % vertices.length].x, 2) + Math.pow(vertices[(i + 2) % vertices.length].y - vertices[(i + 3) % vertices.length].y, 2));
                    if (Math.abs(2.33 * wanted_length - horizontal_length) < horizontal_length * 0.2)
                        return wanted_length;
                    return horizontal_length;
                }
            }
        }

        else if (vertices.length == 5){
            //I think the same thing will work for the 3 vertices case, but I won't put it here yet until we check so not to cause more bugs

            //What if it recognizes 5 vertices but when there should be 6?

            double minX = 0, minY = 0, maxX = 0, maxY = 0;
            for (Point vertex : vertices) {
                minX = Math.min(minX, vertex.x);
                minY = Math.min(minY, vertex.y);
                maxX = Math.min(maxX, vertex.x);
                maxY = Math.min(maxY, vertex.y);
            }

            //we said the order doesn't matter, right? anyway please verify this
            Point[] corrected = {new Point(minX, maxY), new Point(maxX, maxY), new Point(maxX, minY), new Point(minX, minY)};
            return calculatePixels(corrected);
        }

        return -1;
    }
    public double calculateXWithZ(Point[] vertices) {
        double lowest = vertices[0].y;
        for (Point vertex : vertices) {
            if (vertex.y > lowest) {
                lowest = vertex.y;
            }
        }
        return Camera.camera_z / Math.tan(Math.toRadians((lowest - Camera.halfImageHeight) * Camera.vOVERheight));
    }



    //please explain this function
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
