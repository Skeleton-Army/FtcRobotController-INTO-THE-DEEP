package org.firstinspires.ftc.teamcode.utils.autoTeleop;

import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.lowerBlue;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.lowerRed;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.lowerYellow;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.upperBlue;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.upperRed;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.upperYellow;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationHelper;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
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
    private AprilTagProcessor processor;
    private CameraCalibrationIdentity ident;

    boolean viewportPaused;
    public OpenCvCamera webcam;
    private final Telemetry telemetry;
    private Scalar lowerBound = new Scalar(18.4, 66.6, 111.9); // Lower bound for yellow
    private Scalar upperBound = new Scalar(32.6, 255, 255); // Upper bound for yellow

    private final SampleColor color;

    private static final float epsilonConstant = 0.025f;
    private static final Size kernelSize = new Size(5, 5); //We try new one!
    private MecanumDrive drive;
    public static List<Sample> samples = new ArrayList<>();

    public AprilTagSamplesPipeline(AprilTagProcessor processor, Telemetry telemetry, MecanumDrive drive, SampleColor color)
    {
        this.telemetry = telemetry;
        this.drive = drive;
        this.color = color;
        switch (color) {
            case RED:
                this.lowerBound = lowerRed;
                this.upperBound = upperRed;

            case BLUE:
                this.lowerBound = lowerBlue;
                this.upperBound = upperBlue;

            case YELLOW:
                this.lowerBound = lowerYellow;
                this.upperBound = upperYellow;
        }

        this.processor = processor;
        processor.setDecimation(3); // set low decimation to save memory, and there is no need to for any higher :)
    }

    public void noteCalibrationIdentity(CameraCalibrationIdentity ident)
    {
        this.ident = ident;
    }

    @Override
    public void init(Mat firstFrame)
    {
        CameraCalibration calibration = CameraCalibrationHelper.getInstance().getCalibration(ident, firstFrame.width(), firstFrame.height());

        /*calibration.focalLengthX = (float)CameraConfig.fx;
        calibration.focalLengthY = (float)CameraConfig.fy;
        calibration.principalPointX = (float)CameraConfig.cx;
        calibration.principalPointY = (float)CameraConfig.cy;*/

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

    private Mat mask(Mat frame) {
        Mat masked = new Mat();
        // Convert the frame to HSV color space
        Imgproc.cvtColor(frame, masked, Imgproc.COLOR_RGB2YCrCb);

        // Apply color filtering to isolate yellow objects
        Core.inRange(masked, lowerBound, upperBound, masked);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_CLOSE, kernel);
        return masked;
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        List<Sample> samplesFrame = new ArrayList<>();
        List<MatOfPoint> contours = new ArrayList<>();

        //Mat rowsToBlack = input.rowRange(0, THRESHOLD);
        //rowsToBlack.setTo(new Scalar(0, 0, 0));
        Mat masked = mask(input);
        Imgproc.findContours(masked, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Core.bitwise_and(input, masked, input);
        masked.release();

        for (MatOfPoint contour : contours) {
            //check if contour is a valid sample
            //if (contour.size().area() < 500 || contour.size().area() > 5000) //TODO: figure out what these constants should be
            Point lowestPoint = getLowestPoint(contour);
            Sample tempName = new Sample(lowestPoint, drive.pose);
            samplesFrame.add(tempName);

            Imgproc.drawMarker(input, tempName.lowest, new Scalar(255,255,255));
        }


        samples = samplesFrame;
        telemetry.update();
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

        // AprilTag part
        Object drawCtx = processor.processFrame(input, captureTimeNanos);
        requestViewportDrawHook(drawCtx);

        return input;
    }

    // gets robot position based on the first apriltag detection,
    // if didn't one, returns the current robot's pos and hope for the best :)
    public Pose2d getRobotPosByAprilTag() {
        if (!processor.getDetections().isEmpty())  {
            AprilTagDetection detection = processor.getDetections().get(0);
            Position detectionPos = detection.robotPose.getPosition();

            return new Pose2d(detectionPos.x, detectionPos.y, detection.robotPose.getOrientation().getYaw());
        }
        return drive.pose;
    }

    public AprilTagDetection getApriltagDetection() {
        return processor.getDetections().get(0);
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