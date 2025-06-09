package org.firstinspires.ftc.teamcode.utils.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagSamplesPipeline;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

/**
 * This class handles webcam-based computer vision for detecting samples on the field.
 */
public class WebcamCV {
    private final boolean withAprilTag;
    private final boolean onlyAprilTag;
    public OpenCvWebcam webcam;

    OpenCvPipeline pipeline;

    AprilTagProcessor aprilTag;
    AprilTagPipeline aprilTagPipeline;
    static AprilTagSamplesPipeline aprilTagSamplesPipeline;
    static DetectSamples detectSamples;
    List<Sample> samples = new ArrayList<>();
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Follower follower;

    public WebcamCV(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, boolean withAprilTag, boolean onlyAprilTag) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.follower = follower;
        this.withAprilTag = withAprilTag;
        this.onlyAprilTag = onlyAprilTag;
        if (withAprilTag || onlyAprilTag) {
            Position cameraPosition = new Position(DistanceUnit.INCH,
                    CameraConfig.offsetXApriltag, CameraConfig.offsetYApriltag, CameraConfig.offsetZApriltag, 0); //TODO: figure out these!!!
            YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                    CameraConfig.yaw, CameraConfig.offsetVertical,0, 0); //TODO: figure out these!!!
            aprilTag = new AprilTagProcessor.Builder()

                    // The following default settings are available to un-comment and edit as needed.
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
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
            aprilTagPipeline = new AprilTagPipeline(aprilTag, follower);
        }

    }

    /**
     * Computes the distance between a sample and a given position.
     */
    private double distanceFromPosition(Sample currSample, Pose pos) {
        Pose samplePos = currSample.getSamplePosition();
        return Math.sqrt(Math.pow(samplePos.getX() - pos.getX(), 2) + Math.pow(samplePos.getY() - pos.getY(), 2));
    }

    /**
     * Finds the closest sample to the given position.
     * @param pos The reference position.
     * @return Pose2d of the closest sample.
     */
    public Pose getBestSamplePos(Pose pos) {
        return getBestSample(pos).getSamplePosition();
    }

    /**
     * Finds the closest sample to the given position.
     * @param pos The reference position.
     * @return The closest sample.
     */
    public Sample getBestSample(Pose pos) {
        // searching for the min value of distance
        if (samples.isEmpty()) return null;

        Sample closest = samples.get(0);

        for (Sample currSample : samples) {
            if (distanceFromPosition(closest, pos) > distanceFromPosition(currSample, pos)) {
                closest = currSample;
            }
        }

        return closest;
    }

    public Sample getBestSampleInRange(Pose pos, Pose lower, Pose upper) {
        if (samples.isEmpty()) return null;

        Sample closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Sample currSample : samples) {
            Pose samplePos = currSample.getSamplePosition();

            // Ensure sample is within the allowed range
            if (!isLegal(samplePos, lower, upper)) continue;

            double currDistance = distanceFromPosition(currSample, pos);

            if (closest == null || currDistance < minDistance) {
                closest = currSample;
                minDistance = currDistance;
            }
        }

        return closest;
    }

    // checking if the sample is between the boundaries that we can collect
    private boolean isLegal(Pose samplePos, Pose lower, Pose upper) {
        return samplePos.getX() > lower.getX() && samplePos.getX() < upper.getX() && samplePos.getY() > lower.getY() && samplePos.getY() < upper.getY();
    }

    /**
     * Updates the sample list by checking for new samples.
     * @return true if samples are found, false otherwise.
     */
    public boolean lookForSamples() {
        List<Sample> newSamples;

        if (!withAprilTag)
            newSamples = detectSamples.samples;

        else {
            newSamples = AprilTagSamplesPipeline.samples;
        }

        if (!(newSamples.isEmpty())) {
            samples = newSamples;
        }

        return (samples != null) && (!samples.isEmpty());
    }

    /**
     * Configures the webcam for detecting samples with given colors.
     * @param colors The colors to detect.
     */
    public void configureWebcam(SampleColor[] colors) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        OpenCvPipeline selectedPipeline = null;
        if (!onlyAprilTag) {
            if (withAprilTag) {
                if (colors.length == 2)
                    selectedPipeline = new AprilTagSamplesPipeline(aprilTag, telemetry, follower, colors[0], colors[1]);
                if (colors.length == 1)
                    selectedPipeline = new AprilTagSamplesPipeline(aprilTag, telemetry, follower, colors[0]);
                aprilTagSamplesPipeline = (AprilTagSamplesPipeline) selectedPipeline;
            } else if (colors.length == 2)
                selectedPipeline = new DetectSamples(telemetry, webcam, follower, colors[0], colors[1]);
            else
                selectedPipeline = new DetectSamples(telemetry, webcam, follower, colors[0]);
        }
        else
            selectedPipeline = aprilTagPipeline;

        webcam.setPipeline(selectedPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Webcam", "Error: " + errorCode);
            }
        });
    }

    public AprilTagPipeline getAprilTagPipeline() {
        return this.aprilTagPipeline;
    }

    public AprilTagProcessor getAprilTagProcessor() {return this.aprilTag;}


    /**
     * Resets the list of detected samples.
     */
    public void resetSampleList() {
        samples = new ArrayList<>();
    }

    public void stopStream() {
        webcam.stopStreaming();
    }

    public void startStream() {
        webcam.startStreaming(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2, OpenCvCameraRotation.UPRIGHT);
    }
}