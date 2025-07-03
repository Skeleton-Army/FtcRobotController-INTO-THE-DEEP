package org.firstinspires.ftc.teamcode.utils.opencv;


import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.config.cameras.Camera;
import org.firstinspires.ftc.teamcode.utils.config.cameras.CamerasManager;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DetectSamples extends OpenCvPipeline {
    public List<Sample> samples = new ArrayList<>();
    public OpenCvCamera webcam;

    public static Sample targetSample = null;

    private final Telemetry telemetry;
    private final Threshold[] thresholds; // Array of threshold objects for filtering different colors
    private final Follower follower;

    private boolean viewportPaused;

    private static final Size kernelSize = new Size(5, 5);

    public static Mat matrix = new Mat(3, 3, CvType.CV_64F);
    public static MatOfDouble dist;

    private final Mat undistorted = new Mat();
    private final Mat hsv = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
    private final List<MatOfPoint> allContours = new ArrayList<>();
    private final Mat hierarchy = new Mat();
    private final MatOfInt hullIndices = new MatOfInt();
    private final MatOfPoint hullPoints = new MatOfPoint();
    private final MatOfPoint2f ellipsePoints = new MatOfPoint2f();
    private final Mat tempMask = new Mat();

    Camera camera;
    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam, Follower follower, String webcamName,SampleColor color){
        this.telemetry = telemetry;
        this.webcam = webcam;
        this.follower = follower;
        thresholds = new Threshold[] { new Threshold(color) };

        camera = CamerasManager.getByName(webcamName);
        matrix.put(0, 0,
                camera.cameraMatrix);
        dist = new MatOfDouble(camera.distCoeffs);
    }

    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam, Follower follower, String webcamName,SampleColor color1, SampleColor color2){
        this.telemetry = telemetry;
        this.webcam = webcam;
        this.follower = follower;
        thresholds = new Threshold[] { new Threshold(color1), new Threshold(color2) };

        camera = CamerasManager.getByName(webcamName);
        matrix.put(0, 0,
                camera.cameraMatrix);
        dist = new MatOfDouble(camera.distCoeffs);
    }

    /**
     * Finds the lowest point in a given contour.
     * The lowest point is the one with the highest y-value.
     */
    private Point getLowestPoint(MatOfPoint contour) {
        Point[] points = contour.toArray();
        Point lowestPoint = points[0];

        for (Point point : points) {
            if (point.y > lowestPoint.y) {
                lowestPoint = point;
            }
        }

        return lowestPoint;
    }

    /**
     * Processes each frame to detect samples.
     * - Converts the frame to a binary mask based on color thresholds.
     * - Finds contours in the binary image.
     * - Identifies samples and stores their information.
     * - Draws detected contours and markers on the frame.
     */
    public Mat processFrame(Mat input) {
        List<Sample> samplesFrame = new ArrayList<>();
        allContours.clear();

        for (Threshold t : thresholds) {
            // Apply color filtering to isolate the desired objects
            Mat masked = mask(input, t);

            // Create a new list for the current threshold's contours
            List<MatOfPoint> contours = new ArrayList<>();

            // Find contours in the masked image
            Imgproc.findContours(masked, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Add the newly found contours to the master list
            allContours.addAll(contours);

            masked.release(); // Free memory after use

            Scalar color = new Scalar(255, 255, 255);
            switch (t.color) {
                case RED:
                    color = new Scalar(255, 0, 0);
                    break;
                case BLUE:
                    color = new Scalar(0, 0, 255);
                    break;
                case YELLOW:
                    color = new Scalar(255, 255, 0);
                    break;
            }
            Imgproc.drawContours(input, contours, -1, color, -1); // Draw mask
        }

        for (MatOfPoint contour : allContours) {
            Point[] contourArray = contour.toArray();

            if (contourArray.length < 5) {
                continue; // Skip this contour
            }

            Imgproc.convexHull(contour, hullIndices);

            List<Point> hullPointList = new ArrayList<>();
            for (int index : hullIndices.toArray()) {
                hullPointList.add(contourArray[index]);
            }
            hullPoints.fromList(hullPointList);

            // Calculate moments of the convex hull
            Moments moments = Imgproc.moments(hullPoints);

            // Calculate the center of mass (centroid)
            double m00 = moments.get_m00();
            double cx = moments.get_m10() / m00;
            double cy = moments.get_m01() / m00;

            Point center = new Point(cx, cy);

            // Get the lowest point in the detected contour
            Point lowestPoint = getLowestPoint(contour);

            ellipsePoints.fromArray(hullPoints.toArray());
            RotatedRect ellipse = Imgproc.fitEllipse(ellipsePoints);

            // Create and add the new sample
            Sample sample = new Sample(lowestPoint, center, ellipse, follower.getPose());
            sample.calculateArea(Imgproc.boundingRect(contour));
//            Imgproc.putText(input, "(" + Math.round(sample.widthInches * 10) / 10 + ", " + Math.round(sample.heightInches * 10) / 10 + ")", lowestPoint, 0, 1, new Scalar(0, 0, 0));

            if (sample.isTooBig() || sample.isTooSmall()) {
                continue;
            }

            Imgproc.circle(input, center, 5, new Scalar(255, 0, 255));
//            Imgproc.drawMarker(input, lowestPoint, new Scalar(255, 0, 255));
            sample.calculateField();

//            Imgproc.putText(input, "" + sample.orientation, new Point(200, 200), 0, 1, new Scalar(0, 0 ,0));
//            Imgproc.ellipse(input, ellipse, new Scalar(0, 255, 0));
//            double angle = Math.toRadians(90 - ellipse.angle);
//            Imgproc.line(input, lowestPoint, new Point(lowestPoint.x + 50 * Math.cos(angle), lowestPoint.y - 50 * Math.sin(angle)), new Scalar(0, 0, 0));
//            Imgproc.putText(input, "" + ellipse.angle, new Point(20, 20), 0, 1, new Scalar(0, 0, 0));

            samplesFrame.add(sample);
        }

        samples = samplesFrame;

        // Draw contours around detected samples
        //Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

        // Draw the target sample
        if (targetSample != null) {
            Imgproc.circle(input, targetSample.center, 10, new Scalar(0, 255, 255), 3);
            Imgproc.putText(input, "Target", new Point(targetSample.center.x + 12, targetSample.center.y - 12), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
        }

        return input;
    }

    /**
     * Applies color thresholding to isolate the desired objects.
     * - Converts the input frame to YCrCb color space.
     * - Uses threshold values to create a binary mask.
     * - Applies morphological operations to clean up noise.
     */
    private Mat mask(Mat frame, Threshold threshold) {
        // Undistort frame
//        Calib3d.undistort(frame, undistorted, matrix, dist);

        // Convert to HSV
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        // Combine masks from all threshold ranges
        Mat combinedMask = Mat.zeros(hsv.size(), CvType.CV_8UC1);

        for (int i = 0; i < threshold.lowerBounds.size(); i++) {
            Core.inRange(hsv, threshold.lowerBounds.get(i), threshold.upperBounds.get(i), tempMask);
            Core.bitwise_or(combinedMask, tempMask, combinedMask);
        }

        // Apply morphological operations
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

        return combinedMask;
    }

    /**
     * Handles viewport tap events.
     * Toggles between pausing and resuming the camera stream.
     */
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
