package org.firstinspires.ftc.teamcode.utils.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagSamplesPipeline;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class WebcamCV {
    private final boolean withAprilTag;
    public OpenCvWebcam webcam;
    DetectSamples detectSamples;
    AprilTagSamplesPipeline aprilTagSamplesPipeline;

    OpenCvPipeline pipeline;

    AprilTagProcessor aprilTag;
    List<Sample> samples;
    Sample closeSample;
    Vector2d closeSamplePos;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    MecanumDrive drive;

    public WebcamCV(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive, boolean withAprilTag) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.drive = drive;
        this.withAprilTag = withAprilTag;
        if (withAprilTag) {
            Position cameraPosition = new Position(DistanceUnit.INCH,
                    CameraConfig.offsetXApriltag, CameraConfig.offsetYApriltag, CameraConfig.offsetZApriltag, 0); //TODO: figure out these!!!
            YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                    CameraConfig.yaw, CameraConfig.pitch, CameraConfig.roll, 0); //TODO: figure out these!!!
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
        }
    }
    private double distanceFromPosition(Sample currSample, Vector2d pos) {
        Vector2d samplePos = currSample.getSamplePosition().position;
        return Math.pow(samplePos.x - pos.x, 2) + Math.pow(samplePos.y - pos.y, 2);
    }
    public Pose2d getBestSamplePos(Vector2d pos) {
        // searching for the min value of distance
        if (samples.isEmpty())
            return null;
        Sample closest = samples.get(0);

        for (Sample currSample : samples) {
            if (distanceFromPosition(closest, pos) > distanceFromPosition(currSample, pos)) {
                closest = currSample;
            }
        }

        return closest.getSamplePosition();
    }

    public Pose2d getBestOrientation() {
        // searching for the min value of distance
        if (samples.isEmpty())
            return null;
        Sample best = samples.get(0);

        for (Sample currSample : samples) {
            Vector2d pos = currSample.getSamplePosition().position;
            if (pos.x > 6 || pos.x < -6 || pos.y > 18 || pos.y < -15) {
                continue;
            }

            //if (distanceFromPosition(closest, pos) > distanceFromPosition(currSample, pos)) {
            if (Math.abs(currSample.getQuality()) < Math.abs(best.getQuality())) {
                best = currSample;
            }
        }

        return best.getSamplePosition();
    }

    public Sample getCloseSampleObject(Vector2d pos) {
        // searching for the min value of distance
        if (samples.isEmpty())
            return null;
        Sample closest = samples.get(0);

        for (Sample currSample : samples) {
            if (distanceFromPosition(closest, pos) > distanceFromPosition(currSample, pos)) {
                closest = currSample;
            }
        }

        return closest;
    }
    private void printSampleData(Sample inputSample) {
        telemetry.addLine();
        Vector2d samplePos = inputSample.getSamplePosition().position;
        telemetry.addData("x: ", samplePos.x);
        telemetry.addData("y: ", samplePos.y);
        telemetry.addLine();

        telemetry.addData("Point reference: ", closeSample.lowest);
        telemetry.update();
    }
    public void configureWebcam(SampleColor[] colors) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 5);
        if (colors.length == 2)
            detectSamples = new DetectSamples(telemetry, webcam, drive, colors[0], colors[1]);
        if (colors.length == 1)
            detectSamples = new DetectSamples(telemetry, webcam, drive, colors[0]);
        webcam.setPipeline(detectSamples);
        if (withAprilTag && colors.length == 2) {
            aprilTagSamplesPipeline = new AprilTagSamplesPipeline(aprilTag, telemetry, drive, colors[0], colors[1]);
            webcam.setPipeline(aprilTagSamplesPipeline);
            // setting the pipeline to be only samples detector
        }
        if (withAprilTag && colors.length == 1) {
            aprilTagSamplesPipeline = new AprilTagSamplesPipeline(aprilTag, telemetry, drive, colors[0]);
            webcam.setPipeline(aprilTagSamplesPipeline);
            // setting the pipeline to be only samples detector
        }

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Webcam", "Error: " + errorCode);
            }
        });
    }

    public AprilTagSamplesPipeline getAprilTagSamplesPipeline() {
        return this.aprilTagSamplesPipeline;
    }

    public void resetSampleList() {
        samples = new ArrayList<>();
    }
    public void stopStream() {
        webcam.stopStreaming();
    }

    public void startStream() {
        webcam.startStreaming(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2, OpenCvCameraRotation.UPRIGHT);
    }
    public boolean lookForSamples() {
        List<Sample> newSamples;
        if (!withAprilTag)
            newSamples = detectSamples.samples;

        else
            newSamples = AprilTagSamplesPipeline.samples;

        if (!(newSamples.isEmpty())) {
            samples = newSamples;
        }
        return (samples != null) && (!samples.isEmpty());
    }
}