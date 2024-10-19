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

            double[] coordinates = getCoordinates(vertices);
            telemetry.addData("x", coordinates[0]);
            telemetry.addData("y", coordinates[1]);
            double orientation = calculateOrientation(vertices);
            //Imgproc.putText(input, Math.round(coordinates[0]) + "," + Math.round(coordinates[1]), vertices[0], Imgproc.FONT_HERSHEY_SIMPLEX, 0.3, new Scalar(0, 0, 0), 1);
            Imgproc.putText(input, "" + Math.round(Math.toDegrees(orientation)), vertices[0], Imgproc.FONT_HERSHEY_SIMPLEX, 0.3, new Scalar(70, 0, 0), 1);
        }
        telemetry.update();
        return input;
    }


    private Mat mask(Mat frame) {
        Mat masked = new Mat();
        Imgproc.cvtColor(frame, masked, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(masked, lowerBoundMask, upperBoundMask, masked);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_CLOSE, kernel);
        return masked;
    }

    public double[] getCoordinates(Point[] vertices) {
        Point reference = vertices[0];
        for (Point vertex : vertices) {
            if (vertex.y > reference.y) {
                reference = vertex;
            }
        }
        double y = Camera.z / Math.tan(Math.toRadians((reference.y - Camera.halfImageHeight) * Camera.vOVERheight));
        return new double[] {Math.tan(Math.toRadians((reference.x - Camera.halfImageWidth) * Camera.hOVERwidth)) * y, y};
    }

    public double calculateOrientation(Point[] vertices) {
        int index = 0;
        for (int i = 0; i < vertices.length; i++) {
            if (vertices[i].y < vertices[index].y) {
                index = i;
            }
        }
        Point point1 = vertices[(index - 1 + vertices.length) % vertices.length];
        Point point2 = vertices[index];
        Point point3 = vertices[(index + 1) % vertices.length];
        double verticalAngle = (point2.y - Camera.halfImageHeight) * Camera.vOVERheight;
        double wanted_length_squared = Math.pow(point3.y - point2.y, 2) + Math.pow(point2.x - point3.x, 2);
        telemetry.addData("Length 2-3", wanted_length_squared);
        double other_length_squared = Math.pow(point1.y - point2.y, 2) + Math.pow(point2.x - point1.x, 2);
        telemetry.addData("Length 1-2", other_length_squared);
        if (other_length_squared < wanted_length_squared)
            point1 = point3;
        return Math.atan((point1.y - point2.y) / ((point2.x - point1.x) * Math.tan(Math.toRadians(verticalAngle))));
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
