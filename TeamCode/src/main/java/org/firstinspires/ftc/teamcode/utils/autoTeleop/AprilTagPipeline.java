package org.firstinspires.ftc.teamcode.utils.autoTeleop;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationHelper;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

/*
    turns the apriltag processor into a opencv pipeline
    to use it as another pipeline later

 */
public class AprilTagPipeline extends TimestampedOpenCvPipeline
{
    private final MecanumDrive drive;
    private AprilTagProcessor processor;
    private CameraCalibrationIdentity ident;

    public AprilTagPipeline(AprilTagProcessor processor, MecanumDrive drive)
    {
        this.processor = processor;
        this.drive = drive;
    }

    public void noteCalibrationIdentity(CameraCalibrationIdentity ident)
    {
        this.ident = ident;
    }

    @Override
    public void init(Mat firstFrame)
    {
        CameraCalibration calibration = CameraCalibrationHelper.getInstance().getCalibration(ident, firstFrame.width(), firstFrame.height());
        processor.init(firstFrame.width(), firstFrame.height(), calibration);
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        Object drawCtx = processor.processFrame(input, captureTimeNanos);
        requestViewportDrawHook(drawCtx);
        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        processor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }

    // gets robot position based on the first apriltag detection,
    // if didn't one, returns the current robot's pos and hope for the best :)
    public Pose2d getRobotPosByAprilTag() {
        if (!processor.getDetections().isEmpty())  {
            AprilTagDetection detection = processor.getDetections().get(0);
            Position detectionPos = detection.robotPose.getPosition();

            drive.pose = new Pose2d(detectionPos.x, detectionPos.y, Math.toRadians(detection.robotPose.getOrientation().getYaw() + 90));
            return new Pose2d(detectionPos.x, detectionPos.y, Math.toRadians(detection.robotPose.getOrientation().getYaw() + 90));
        }
        return drive.pose;
    }

    public AprilTagDetection getApriltagDetection() {
        if (!processor.getDetections().isEmpty())
            return processor.getDetections().get(0);
        return null;
    }

}