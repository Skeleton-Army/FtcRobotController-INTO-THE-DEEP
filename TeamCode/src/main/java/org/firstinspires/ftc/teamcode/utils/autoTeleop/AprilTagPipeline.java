package org.firstinspires.ftc.teamcode.utils.autoTeleop;

import android.graphics.Canvas;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationHelper;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.teamcode.utils.config.cameras.Camera;
import org.firstinspires.ftc.teamcode.utils.config.cameras.CamerasManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

/*
    turns the apriltag processor into a opencv pipeline
    to use it as another pipeline later

 */
public class AprilTagPipeline extends TimestampedOpenCvPipeline
{
    private final Follower follower;
    private AprilTagProcessor processor;
    private CameraCalibrationIdentity ident;

    Mat undistored = new Mat();
    Mat matrix = new Mat(3, 3, CvType.CV_64F);
    MatOfDouble dist;
    public AprilTagPipeline(AprilTagProcessor processor, Follower follower, String webcamName)
    {
        this.processor = processor;
        this.follower = follower;

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
        processor.init(firstFrame.width(), firstFrame.height(), calibration);
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        Calib3d.undistort(input, undistored, matrix, dist);
        Object drawCtx = processor.processFrame(undistored, captureTimeNanos);
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

}