package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.cameraMatrix;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.distCoeffs;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
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

    private final Telemetry telemetry;
    private final Threshold[] thresholds; // Array of threshold objects for filtering different colors
    private final MecanumDrive drive;

    private boolean viewportPaused;

    // Adjusted yellow HSV bounds
    private static final float epsilonConstant = 0.025f;
    private static final Size kernelSize = new Size(5, 5);

    private static Mat input;
    Mat matrix = new Mat(3, 3, CvType.CV_64F);

    Point second;
    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam, MecanumDrive drive, SampleColor color){
        this.telemetry = telemetry;
        this.webcam = webcam;
        this.drive = drive;
        thresholds = new Threshold[] { new Threshold(color) };

        matrix.put(0, 0,
                cameraMatrix[0], cameraMatrix[1], cameraMatrix[2],
                cameraMatrix[3], cameraMatrix[4], cameraMatrix[5],
                cameraMatrix[6], cameraMatrix[7], cameraMatrix[8]);
    }

    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam, MecanumDrive drive, SampleColor color1, SampleColor color2){
        this.telemetry = telemetry;
        this.webcam = webcam;
        this.drive = drive;
        thresholds = new Threshold[] { new Threshold(color1), new Threshold(color2) };

        matrix.put(0, 0,
                cameraMatrix[0], cameraMatrix[1], cameraMatrix[2],
                cameraMatrix[3], cameraMatrix[4], cameraMatrix[5],
                cameraMatrix[6], cameraMatrix[7], cameraMatrix[8]);
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
        List<MatOfPoint> allContours = new ArrayList<>();

        DetectSamples.input = input;
//        Mat input2 = input.clone();

        for (Threshold t : thresholds) {
            // Apply color filtering to isolate the desired objects
            Mat masked = mask(input, t);

            // Create a new list for the current threshold's contours
            List<MatOfPoint> contours = new ArrayList<>();

            // Find contours in the masked image
            Imgproc.findContours(masked, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Add the newly found contours to the master list
            allContours.addAll(contours);

            masked.release(); // Free memory after use

//            Imgproc.drawContours(input2, contours, -1, new Scalar(255, 255, 255), -1); // Draw mask
        }

        for (MatOfPoint contour : allContours) {
            MatOfInt hullIndices = new MatOfInt();
            Imgproc.convexHull(contour, hullIndices);

            MatOfPoint hullPoints = new MatOfPoint();
            List<Point> hullPointList = new ArrayList<>();
            Point[] contourArray = contour.toArray();
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

            if (contour.toArray().length < 5) {
                continue; // Skip this contour
            }

            RotatedRect ellipse = Imgproc.fitEllipse(new MatOfPoint2f(hullPoints.toArray()));

            // Create and add the new sample
            Sample sample = new Sample(lowestPoint, center, ellipse, drive.pose);
            sample.calculateArea(Imgproc.boundingRect(contour));
//            Imgproc.putText(input, "(" + Math.round(sample.widthInches * 10) / 10 + ", " + Math.round(sample.heightInches * 10) / 10 + ")", lowestPoint, 0, 1, new Scalar(0, 0, 0));
//            Imgproc.circle(input, center, 1, new Scalar(255, 0, 0));

            if (sample.isTooBig() || sample.isTooSmall()) {
                continue;
            }

            Imgproc.drawMarker(input, lowestPoint, new Scalar(255, 0, 255));
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
        MatOfDouble dist = new MatOfDouble(distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4]);
        Mat undistorted = new Mat();
        Calib3d.undistort(frame, undistorted, matrix, dist);

        // Convert to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(undistorted, hsv, Imgproc.COLOR_RGB2HSV);
        undistorted.release();

        // Combine masks from all threshold ranges
        Mat combinedMask = Mat.zeros(hsv.size(), CvType.CV_8UC1);

        for (int i = 0; i < threshold.lowerBounds.size(); i++) {
            Mat tempMask = new Mat();
            Core.inRange(hsv, threshold.lowerBounds.get(i), threshold.upperBounds.get(i), tempMask);
            Core.bitwise_or(combinedMask, tempMask, combinedMask);
            tempMask.release();
        }

        hsv.release();

        // Apply morphological operations
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
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
