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
import org.opencv.core.Rect;
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

    Point second;
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

    private Vector2d pixelToWorld(double x, double y) {
        double horizontal = Math.toRadians((CameraConfig.halfImageWidth - x) * CameraConfig.hOverWidth() + CameraConfig.offsetHorizontal);
        double worldY = CameraConfig.z / Math.tan(Math.toRadians((y - CameraConfig.halfImageHeight) * CameraConfig.vOverHeight() + CameraConfig.offsetVertical));
        double worldX = Math.tan(horizontal) * worldY;
        return new Vector2d(worldX, worldY);
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
            RotatedRect ellipse = Imgproc.fitEllipse(new MatOfPoint2f(contour.toArray()));
            // Create and add the new sample
            Sample sample = new Sample(lowestPoint, center, ellipse, drive.pose);
            //sample.calculateOrientation(contour);
            sample.calculateArea(Imgproc.boundingRect(contour));
            Imgproc.putText(input, "(" + Math.round(sample.widthInches * 10) / 10 + ", " + Math.round(sample.heightInches * 10) / 10 + ")", lowestPoint, 0, 1, new Scalar(0, 0, 0));
            Imgproc.circle(input, center, 1, new Scalar(255, 0, 0));
            if (sample.isTooBig()) {
                continue;
            }
            Imgproc.drawMarker(input, lowestPoint, new Scalar(255, 0, 255));
            sample.calculateField();
            Imgproc.putText(input, "" + sample.orientation, new Point(200, 200), 0, 1, new Scalar(0, 0 ,0));
            Imgproc.ellipse(input, ellipse, new Scalar(0, 255, 0));
            double angle = Math.toRadians(90 - ellipse.angle);
            Imgproc.line(input, lowestPoint, new Point(lowestPoint.x + 30 * Math.cos(angle), lowestPoint.y - 30 * Math.sin(angle)), new Scalar(0, 0, 0));
            Imgproc.putText(input, "" + ellipse.angle, new Point(20, 20), 0, 1, new Scalar(0, 0, 0));
            samplesFrame.add(sample);

            // Draw a marker on the detected point

//            Vector2d secondvec = pixelToWorld(lowestPoint.x + 40 * Math.cos(Math.toRadians(rotated.angle)), lowestPoint.y - 40 * Math.sin(Math.toRadians(rotated.angle)));
//            second = new Point(lowestPoint.x + 40 * Math.cos(Math.toRadians(rotated.angle)), lowestPoint.y - 40 * Math.sin(Math.toRadians(rotated.angle)));
//
//
//
//            Imgproc.line(input, lowestPoint, second, new Scalar(0,0,255));
//            Imgproc.rectangle(input, rotated.boundingRect(), new Scalar(255,0,0));
//            Imgproc.putText(input, "" + rotated.angle, lowestPoint, 0, 1, new Scalar(255, 0, 255));
//            Imgproc.putText(input, "" + sample.orientation, new Point(200, 100), 0, 1, new Scalar(255, 0, 255));
//            Imgproc.drawMarker(input, lowestPoint, new Scalar(255,255,0));

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
