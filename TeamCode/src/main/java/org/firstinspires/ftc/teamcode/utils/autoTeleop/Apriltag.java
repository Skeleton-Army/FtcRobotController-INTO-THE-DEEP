package org.firstinspires.ftc.teamcode.utils.autoTeleop;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Apriltag {
    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            CameraConfig.offsetXApriltag, CameraConfig.offsetYApriltag, CameraConfig.offsetZApriltag, 0); //TODO: figure out these!!!
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            CameraConfig.yaw, CameraConfig.pitch, CameraConfig.roll, 0); //TODO: figure out these!!!


    List<AprilTagDetection> currentDetections;

    HardwareMap hardwareMap;

    static MecanumDrive drive;

    public Apriltag(HardwareMap hardwareMap, MecanumDrive drive) {
        this.hardwareMap = hardwareMap;
        this.drive = drive;

        initAprilTag();
        disableApriltag();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
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
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        currentDetections = aprilTag.getDetections();

    }   // end method initAprilTag()

    public static Pose2d getRobotPos(AprilTagDetection aprilTagDetection) {
        Position robotPose = aprilTagDetection.robotPose.getPosition();
        double robotAngle = aprilTagDetection.robotPose.getOrientation().getYaw();

        return new Pose2d(robotPose.x, robotPose.y, robotAngle);
    }

    public static void updateRobotPos(AprilTagDetection aprilTagDetection) {
        drive.pose = getRobotPos(aprilTagDetection);
    }

    public static List<AprilTagDetection> getCurrentDetections() {
        return aprilTag.getDetections();
    }

    public void enableApriltag() {
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    public void disableApriltag() {
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

}
