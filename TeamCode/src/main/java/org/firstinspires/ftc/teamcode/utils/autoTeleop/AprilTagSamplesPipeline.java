package org.firstinspires.ftc.teamcode.utils.autoTeleop;

import android.graphics.Canvas;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationHelper;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.teamcode.utils.config.cameras.Camera;
import org.firstinspires.ftc.teamcode.utils.config.cameras.CamerasManager;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.teamcode.utils.opencv.Threshold;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
    turns the apriltag processor into a opencv pipeline
    to use it as another pipeline later

 */
public class AprilTagSamplesPipeline extends TimestampedOpenCvPipeline
{
    private Threshold[] thresholds;
    private AprilTagProcessor processor;
    private CameraCalibrationIdentity ident;

    boolean viewportPaused;
    public static List<Sample> samples = new ArrayList<>();
    public OpenCvCamera webcam;

    public static Sample targetSample = null;

    private final Telemetry telemetry;
        private final Follower follower;

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
    public AprilTagSamplesPipeline(AprilTagProcessor processor,Telemetry telemetry, Follower follower, String webcamName, SampleColor color){
        this.telemetry = telemetry;
        this.webcam = webcam;
        this.follower = follower;
        thresholds = new Threshold[] {new Threshold(color)};
        this.processor = processor;

        Camera targetCamera = CamerasManager.getByName(webcamName);

        matrix.put(0, 0,
                targetCamera.cameraMatrix[0], targetCamera.cameraMatrix[1], targetCamera.cameraMatrix[2],
                targetCamera.cameraMatrix[3], targetCamera.cameraMatrix[4], targetCamera.cameraMatrix[5],
                targetCamera.cameraMatrix[6], targetCamera.cameraMatrix[7], targetCamera.cameraMatrix[8]);
        dist = new MatOfDouble(targetCamera.distCoeffs[0], targetCamera.distCoeffs[1], targetCamera.distCoeffs[2], targetCamera.distCoeffs[3], targetCamera.distCoeffs[4]);
    }

    public AprilTagSamplesPipeline(AprilTagProcessor processor, Telemetry telemetry, Follower follower, String webcamName, SampleColor color1, SampleColor color2)
    {
        this.telemetry = telemetry;
        this.follower = follower;
        thresholds = new Threshold[] { new Threshold(color1), new Threshold(color2) };

        this.processor = processor;
        processor.setDecimation(3); // set low decimation to save memory, and there is no need to for any higher :)
        Camera targetCamera = CamerasManager.getByName(webcamName);

        matrix.put(0, 0,
                targetCamera.cameraMatrix[0], targetCamera.cameraMatrix[1], targetCamera.cameraMatrix[2],
                targetCamera.cameraMatrix[3], targetCamera.cameraMatrix[4], targetCamera.cameraMatrix[5],
                targetCamera.cameraMatrix[6], targetCamera.cameraMatrix[7], targetCamera.cameraMatrix[8]);
        dist = new MatOfDouble(targetCamera.distCoeffs[0], targetCamera.distCoeffs[1], targetCamera.distCoeffs[2], targetCamera.distCoeffs[3], targetCamera.distCoeffs[4]);
    }

    public void noteCalibrationIdentity(CameraCalibrationIdentity ident)
    {
        this.ident = ident;
    }

    @Override
    public void init(Mat firstFrame)
    {
        CameraCalibration calibration = CameraCalibrationHelper.getInstance().getCalibration(ident, firstFrame.width(), firstFrame.height());

//        calibration.focalLengthX = (float)CameraConfig.fx;
//        calibration.focalLengthY = (float) CameraConfig.fy;
//        calibration.principalPointX = (float)CameraConfig.cx;
//        calibration.principalPointY = (float)CameraConfig.cy;

        processor.init(firstFrame.width(), firstFrame.height(), calibration);
    }

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
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
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

        // AprilTag part
        Object drawCtx = processor.processFrame(input, captureTimeNanos);
        requestViewportDrawHook(drawCtx);

        return input;
    }

    // gets robot position based on the first apriltag detection,
    // if didn't one, returns the current robot's pos and hope for the best :)
    public Pose getRobotPosByAprilTag() {
        if (!processor.getDetections().isEmpty())  {
            AprilTagDetection detection = processor.getDetections().get(0);
            Position detectionPos = detection.robotPose.getPosition();

            follower.setPose(new Pose(detectionPos.x, detectionPos.y, Math.toRadians(detection.robotPose.getOrientation().getYaw() + 90)));
            return new Pose(detectionPos.x, detectionPos.y, Math.toRadians(detection.robotPose.getOrientation().getYaw() + 90));
        }
        return follower.getPose();
    }

    public AprilTagDetection getApriltagDetection() {
        if (!processor.getDetections().isEmpty())
            return processor.getDetections().get(0);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        processor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
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