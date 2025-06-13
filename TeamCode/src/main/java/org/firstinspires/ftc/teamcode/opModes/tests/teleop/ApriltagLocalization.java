package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.config.cameras.Camera;
import org.firstinspires.ftc.teamcode.utils.config.cameras.CamerasManager;
import org.firstinspires.ftc.teamcode.utils.general.Drawing;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Apriltag Localization", group = "test")
public class ApriltagLocalization extends OpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    String WebcamName = "Webcam 1";
    Camera camera = CamerasManager.getByName(WebcamName);
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            camera.offsetX, camera.offsetY, camera.offsetZ, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            camera.yaw, camera.pitch, camera.roll, 0); //TODO: figure out these!!!

    private int decimation = 1;
    private final int decimationMin = 1;
    private final int decimationMax = 5;

    boolean thisDecimationUp = false;
    boolean thisDecimationDn = false;
    boolean lastDecimationDn = false;
    boolean lastDecimationUp = false;
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
                        camera.fx,
                        camera.fy,
                        camera.cx,
                        camera.cy
                )
                .setNumThreads(3)
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
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(camera.width, camera.height));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

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

    }   // end method initAprilTag()

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine("----------------");
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine("----------------");
                telemetry.addLine("relative position to the camera: ");
                telemetry.addData("x: ",detection.rawPose.x);
                telemetry.addData("y: ",detection.rawPose.y);
                telemetry.addData("z: ",detection.rawPose.z);
                telemetry.addLine("----------------");
                telemetry.addData("bearing ",detection.ftcPose.bearing);
                telemetry.addData("range ",detection.ftcPose.range);
                telemetry.addData("yaw ",detection.ftcPose.yaw);
                telemetry.addLine("----------------");
                telemetry.addData("Confidence: ", detection.decisionMargin);
                telemetry.addLine("----------------");
                telemetry.addData("Current decimation: ", decimation);
                telemetry.addLine("----------------");
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                telemetry.addLine("----------------");

                telemetry.addData("solve time: ", aprilTag.getPerTagAvgPoseSolveTime());
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, Math.toRadians(detection.robotPose.getOrientation().getYaw())), "#3F51B5");
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }
    @Override
    public void init() {
        initAprilTag();
    }

    @Override
    public void init_loop() {
        telemetryAprilTag();

        thisDecimationUp = gamepad1.dpad_up;
        thisDecimationDn = gamepad1.dpad_down;


        if (thisDecimationUp && !lastDecimationUp) {
            decimation = Range.clip(decimation + 1, decimationMin, decimationMax);
            aprilTag.setDecimation(decimation);
        }

        if (thisDecimationDn && !lastDecimationDn) {
            decimation = Range.clip(decimation - 1, decimationMin, decimationMax);
            aprilTag.setDecimation(decimation);
        }

        lastDecimationUp = thisDecimationUp;
        lastDecimationDn = thisDecimationDn;
    }
    @Override
    public void loop() {

    }
}
