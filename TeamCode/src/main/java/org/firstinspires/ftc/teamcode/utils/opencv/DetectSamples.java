package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.cameraMatrix;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.distCoeffs;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
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

    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam, MecanumDrive drive, SampleColor color){
        this.telemetry = telemetry;
        this.webcam = webcam;
        this.drive = drive;
        thresholds = new Threshold[] { new Threshold(color) };
    }

    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam, MecanumDrive drive, SampleColor color1, SampleColor color2){
        this.telemetry = telemetry;
        this.webcam = webcam;
        this.drive = drive;
        thresholds = new Threshold[] { new Threshold(color1), new Threshold(color2) };
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
        List<MatOfPoint> contours = new ArrayList<>();

        DetectSamples.input = input;

        //Mat rowsToBlack = input.rowRange(0, THRESHOLD);
        //rowsToBlack.setTo(new Scalar(0, 0, 0));

        // Apply color filtering to isolate the desired objects
        Mat masked = mask(input);

        // Find contours in the masked image
        Imgproc.findContours(masked, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Core.bitwise_and(input, masked, input);

        masked.release(); // Free memory after use

        for (MatOfPoint contour : contours) {
            //check if contour is a valid sample
            //if (contour.size().area() < 500 || contour.size().area() > 5000) //TODO: figure out what these constants should be

            // Get the lowest point in the detected contour
            Point lowestPoint = getLowestPoint(contour);

            // Create and add the new sample
            Sample sample = new Sample(lowestPoint, drive.pose, Imgproc.minAreaRect(new MatOfPoint2f(contour)));
            //sample.calculateOrientation(contour);
            sample.calculateField();

            samplesFrame.add(sample);

            // Draw a marker on the detected point
            Imgproc.drawMarker(input, new Point(lowestPoint.x, lowestPoint.y), new Scalar(255,255,0));
        }

        samples = samplesFrame;

        // Log sample coordinates to telemetry for debugging
        if (!samplesFrame.isEmpty()) {
            telemetry.addData("Image X Coordinate:", samplesFrame.get(0).lowest.x);
            telemetry.addData("Image Y Coordinate:", samplesFrame.get(0).lowest.y);
        }

        // Draw contours around detected samples
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

        return input;
    }

    /**
     * Applies color thresholding to isolate the desired objects.
     * - Converts the input frame to YCrCb color space.
     * - Uses threshold values to create a binary mask.
     * - Applies morphological operations to clean up noise.
     */
    private Mat mask(Mat frame) {
        // Undistort frame
        Mat matrix = new Mat(3, 3, CvType.CV_64F);
        matrix.put(0, 0,
                cameraMatrix[0], cameraMatrix[1], cameraMatrix[2],
                cameraMatrix[3], cameraMatrix[4], cameraMatrix[5],
                cameraMatrix[6], cameraMatrix[7], cameraMatrix[8]);

        MatOfDouble dist = new MatOfDouble(distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4]);

        Mat undistorted = new Mat();
        Calib3d.undistort(frame, undistorted, matrix, dist);

        // Create mask
        Mat masked = new Mat();

        // Convert the frame to HSV color space
        Imgproc.cvtColor(undistorted, masked, Imgproc.COLOR_RGB2YCrCb);

        Mat binary = Mat.zeros(undistorted.size(), Imgproc.THRESH_BINARY);

        // Apply color filtering to isolate the desired objects
        for (Threshold t : thresholds) {
            Core.inRange(masked, t.lowerBound, t.upperBound, binary);
        }

        masked.release(); // Free memory after use
        undistorted.release(); // Free memory after use

        // Apply morphological operations to remove noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_OPEN, kernel); // Removes small noise
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_CLOSE, kernel); // Closes small gaps

        return binary;
    }

    public static void drawSample(MatOfPoint contour) {
        List<MatOfPoint> contours = new ArrayList<>();
        contours.add(0, contour);
        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0));
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
