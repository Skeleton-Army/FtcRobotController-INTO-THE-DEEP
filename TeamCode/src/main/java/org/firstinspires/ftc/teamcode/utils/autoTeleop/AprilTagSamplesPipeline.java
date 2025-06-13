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
    public OpenCvCamera webcam;
    private final Telemetry telemetry;
    private static Mat input;

    private static final float epsilonConstant = 0.025f;
    private static final Size kernelSize = new Size(5, 5); //We try new one!
    private Follower follower;
    public static List<Sample> samples = new ArrayList<>();
    Mat matrix = new Mat(3, 3, CvType.CV_64F);

    MatOfDouble dist;
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
        // Undistort frame


        Mat undistorted = new Mat();
        Calib3d.undistort(frame, undistorted, matrix, dist);

        // Create mask
        Mat masked = new Mat();

        // Convert the frame to HSV color space
        Imgproc.cvtColor(undistorted, masked, Imgproc.COLOR_RGB2YCrCb);

        Mat binary = Mat.zeros(undistorted.size(), Imgproc.THRESH_BINARY);

        // Apply color filtering to isolate the desired objects
        Core.inRange(masked, threshold.lowerBound, threshold.upperBound, binary);

        masked.release(); // Free memory after use
        undistorted.release(); // Free memory after use

        // Apply morphological operations to remove noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_OPEN, kernel); // Removes small noise
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_CLOSE, kernel); // Closes small gaps

        return binary;
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        List<Sample> samplesFrame = new ArrayList<>();
        List<MatOfPoint> allContours = new ArrayList<>();

        AprilTagSamplesPipeline.input = input;

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