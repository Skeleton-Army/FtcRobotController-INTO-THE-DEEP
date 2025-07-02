package org.firstinspires.ftc.teamcode.utils.opencv;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.utils.config.cameras.Camera;
import org.firstinspires.ftc.teamcode.utils.config.cameras.CamerasManager;
import org.firstinspires.ftc.vision.VisionProcessor;
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

import java.util.ArrayList;
import java.util.List;

public class DetectSamplesProcessor implements VisionProcessor, CameraStreamSource {

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
    private final Mat combinedMask = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
    private final List<MatOfPoint> allContours = new ArrayList<>();
    private final Mat hierarchy = new Mat();
    private final MatOfInt hullIndices = new MatOfInt();
    private final MatOfPoint hullPoints = new MatOfPoint();
    private final MatOfPoint2f ellipsePoints = new MatOfPoint2f();
    public DetectSamplesProcessor(Telemetry telemetry, Follower follower,String webcamName, SampleColor color){
        this.telemetry = telemetry;
        this.follower = follower;
        thresholds = new Threshold[] { new Threshold(color) };

        Camera targetCamera = CamerasManager.getByName(webcamName);
        matrix.put(0, 0,
                targetCamera.cameraMatrix[0], targetCamera.cameraMatrix[1], targetCamera.cameraMatrix[2],
                targetCamera.cameraMatrix[3], targetCamera.cameraMatrix[4], targetCamera.cameraMatrix[5],
                targetCamera.cameraMatrix[6], targetCamera.cameraMatrix[7], targetCamera.cameraMatrix[8]);
        dist = new MatOfDouble(targetCamera.distCoeffs[0], targetCamera.distCoeffs[1], targetCamera.distCoeffs[2], targetCamera.distCoeffs[3], targetCamera.distCoeffs[4]);
    }

    public DetectSamplesProcessor(Telemetry telemetry, Follower follower,String webcamName, SampleColor color1, SampleColor color2){
        this.telemetry = telemetry;
        this.follower = follower;
        thresholds = new Threshold[] { new Threshold(color1), new Threshold(color2) };

        Camera targetCamera = CamerasManager.getByName(webcamName);
        matrix.put(0, 0,
                targetCamera.cameraMatrix[0], targetCamera.cameraMatrix[1], targetCamera.cameraMatrix[2],
                targetCamera.cameraMatrix[3], targetCamera.cameraMatrix[4], targetCamera.cameraMatrix[5],
                targetCamera.cameraMatrix[6], targetCamera.cameraMatrix[7], targetCamera.cameraMatrix[8]);
        dist = new MatOfDouble(targetCamera.distCoeffs[0], targetCamera.distCoeffs[1], targetCamera.distCoeffs[2], targetCamera.distCoeffs[3], targetCamera.distCoeffs[4]);
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

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        // this method comes with all VisionProcessors, we just don't need to do anything here, and you dont need to call it
    }


    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        List<Sample> samplesFrame = new ArrayList<>();
        allContours.clear();  // Clear previous frame's contours

        for (Threshold t : thresholds) {
            // Apply color filtering to isolate the desired objects
            Mat masked = mask(input, t);

            // Create a new list for the current threshold's contours
            List<MatOfPoint> contours = new ArrayList<>();

            // Find contours in the masked image
            Imgproc.findContours(masked, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Add the newly found contours to the master list
            allContours.addAll(contours);

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

            masked.release();
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

        // Draw the target sample
        if (targetSample != null) {
            Imgproc.circle(input, targetSample.center, 10, new Scalar(0, 255, 255), 3);
            Imgproc.putText(input, "Target", new Point(targetSample.center.x + 12, targetSample.center.y - 12), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
        }

        for (MatOfPoint c : allContours) {
            c.release();
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
        Calib3d.undistort(frame, undistorted, matrix, dist);
        Imgproc.cvtColor(undistorted, hsv, Imgproc.COLOR_RGB2HSV);

        combinedMask.setTo(new Scalar(0));

        for (int i = 0; i < threshold.lowerBounds.size(); i++) {
            Mat tempMask = new Mat();
            Core.inRange(hsv, threshold.lowerBounds.get(i), threshold.upperBounds.get(i), tempMask);
            Core.bitwise_or(combinedMask, tempMask, combinedMask);
            tempMask.release();
        }

        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

        return combinedMask.clone(); // safe copy, avoids future bugs
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    @Override
    protected void finalize() throws Throwable {

    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        //continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}