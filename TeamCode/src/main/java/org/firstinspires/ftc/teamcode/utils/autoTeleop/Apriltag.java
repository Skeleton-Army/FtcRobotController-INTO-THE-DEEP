package org.firstinspires.ftc.teamcode.utils.autoTeleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Apriltag {
    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            CameraConfig.offsetXApriltag, CameraConfig.offsetYApriltag, CameraConfig.offsetZApriltag, 0); //TODO: figure out these!!!
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            CameraConfig.yaw, CameraConfig.offsetVertical, 0, 0); //TODO: figure out these!!!


    List<AprilTagDetection> currentDetections;

    HardwareMap hardwareMap;

    static Follower follower;

    public Apriltag(HardwareMap hardwareMap, Follower follower) {
        this.hardwareMap = hardwareMap;
        this.follower = follower;

        initAprilTag(); // creates the apriltag processor
        //disableApriltag();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(true)
                //.setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(
                        CameraConfig.fx,
                        CameraConfig.fy,
                        CameraConfig.cx,
                        CameraConfig.cy
                )
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(1413.91, 1413.91, 965.446, 529.378)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        //VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        //builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        //builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        //visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        currentDetections = aprilTag.getDetections();

    }   // end method initAprilTag()

    public static List<AprilTagDetection> getCurrentDetections() {
        return aprilTag.getDetections();
    }

    public AprilTagProcessor getAprilTagAprocessor() {
        return aprilTag;
    }

    public Pose getRobotPosByAprilTag() {
        if (!aprilTag.getDetections().isEmpty())  {
            AprilTagDetection detection = aprilTag.getDetections().get(0);
            Position detectionPos = detection.robotPose.getPosition();

            follower.setPose(new Pose(detectionPos.x, detectionPos.y, Math.toRadians(detection.robotPose.getOrientation().getYaw() + 90)));
            return new Pose(detectionPos.x, detectionPos.y, Math.toRadians(detection.robotPose.getOrientation().getYaw() + 90));
        }
        return follower.getPose();
    }

    public AprilTagDetection getApriltagDetection() {
        if (!aprilTag.getDetections().isEmpty())
            return aprilTag.getDetections().get(0);
        return null;
    }
    public void enableApriltag() {
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    public void disableApriltag() {
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    public void debug(Telemetry telemetry) {

        telemetry.addData("solve time: ",aprilTag.getPerTagAvgPoseSolveTime());
        telemetry.addData("detections: ",aprilTag.getDetections());
        telemetry.addData("fresh detections: ",aprilTag.getFreshDetections());
        //telemetry.addData("fresh detections: ",aprilTag.processFrame());

        telemetry.update();
    }
}
