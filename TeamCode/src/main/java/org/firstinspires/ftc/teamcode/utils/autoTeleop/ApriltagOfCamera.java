package org.firstinspires.ftc.teamcode.utils.autoTeleop;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


// apriltag class for all cameras to use.
// this will create a apriltag processor for the camera
public class ApriltagOfCamera {

    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;

    Position cameraPosition;
    public AprilTagProcessor aprilTag;
    YawPitchRollAngles cameraOrientation;
    public ApriltagOfCamera(double fx, double fy,double cx,double cy, Position cameraPosition, YawPitchRollAngles cameraOrientation) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.cameraPosition = cameraPosition;
        this.cameraOrientation = cameraOrientation;

        initAprilTag();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(
                        fx,
                        fy,
                        cx,
                        cy
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



    }
}
