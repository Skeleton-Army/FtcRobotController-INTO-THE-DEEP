package org.firstinspires.ftc.teamcode.utils.opencv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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

            // Get the bounding box for the contour to determine its center X
            Rect boundingBox = Imgproc.boundingRect(contour);
            int center = boundingBox.x + boundingBox.width / 2;

            // Create and add the new sample
            Sample sample = new Sample(new Point(center, lowestPoint.y), drive.pose);
            sample.calculateOrientation(contour);
            sample.calculateField();

            samplesFrame.add(sample);

            // Draw a marker on the detected point
            Imgproc.drawMarker(input, new Point(center, lowestPoint.y), new Scalar(255,255,0));
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
        Mat masked = new Mat();

        // Convert the frame to HSV color space
        Imgproc.cvtColor(frame, masked, Imgproc.COLOR_RGB2YCrCb);

        Mat binary = Mat.zeros(frame.size(), Imgproc.THRESH_BINARY);

        // Apply color filtering to isolate yellow objects
        for (Threshold t : thresholds) {
            Core.inRange(masked, t.lowerBound, t.upperBound, binary);
        }

        masked.release(); // Free memory after use

        // Apply morphological operations to remove noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_OPEN, kernel); // Removes small noise
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_CLOSE, kernel); // Closes small gaps

        return binary;
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
