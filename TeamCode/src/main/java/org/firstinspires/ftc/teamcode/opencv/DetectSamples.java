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

    //Raviv: general question - in most Imgproc functions there is a destination matrix,
    // that we create a new one most of the time for but then never use the original again,
    // will there be a problem if we use the same matrix in the dst as the origin?
    // because I think it will save a lot of (slow) matrix creations

    public OpenCvCamera webcam;
    boolean viewportPaused; //Do we really need this?
    private final Telemetry telemetry;

    //yellow
    private final Scalar lowerBoundMask = new Scalar(0, 138, 0);
    private final Scalar upperBoundMask = new Scalar(255, 200, 100);

    private static final float epsilonConstant = 0.025f;
    private static final Size kernelSize = new Size(5, 5); //Why did we choose this value?


    public DetectSamples(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public Mat processFrame(Mat input) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask(input), contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            double epsilon = epsilonConstant * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, contour2f, epsilon, true);
            Point[] vertices = contour2f.toArray();

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
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(frame, lowerBoundMask, upperBoundMask, frame);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
        Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_CLOSE, kernel);
        return frame;
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
            for (int i = 0; i < vertices.length; i++) {
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
            //Raviv: did we encounter that case?

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
        return Camera.z / Math.tan(Math.toRadians((lowest - Camera.halfImageHeight) * Camera.vOVERheight));
    }
    public double[] calculatePixelLengthAndOrientation(Point[] vertices) {
        int index = 0;
        for (int i = 0; i < vertices.length; i++) {
            if (vertices[i].y < vertices[index].y) {
                index = i;
            }
        }
        Point point1 = vertices[(index -1) % vertices.length];
        Point point2 = vertices[index];
        Point point3 = vertices[(index + 1) % vertices.length];
        double verticalAngle = (vertices[index].y - Camera.halfImageHeight) * Camera.vOVERheight;
        double Orientation = Math.atan((point1.y - point2.y) / ((point1.x - point2.x) * Math.tan(Math.toRadians(verticalAngle))));
        double wanted_length = (point2.x - point1.x) / (Math.tan(Math.toRadians(verticalAngle)) * Math.sin(Orientation));
        if (wanted_length < (point2.x - point3.x) / (Math.tan(Math.toRadians(verticalAngle)) * Math.sin(90 - Orientation))) {
            Orientation = Orientation - 90;
            wanted_length *= 2.33;
        }
        return new double[] {wanted_length / 2.33, Math.toDegrees(Orientation)};
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
