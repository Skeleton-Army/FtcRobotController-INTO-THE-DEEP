package org.firstinspires.ftc.teamcode.utils.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagSamplesPipeline;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
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
    }
    private double distanceFromPosition(Sample currSample, Vector2d pos) {
        Vector2d samplePos = currSample.getSamplePosition();
        return Math.pow(samplePos.x - pos.x, 2) + Math.pow(samplePos.y - pos.y, 2);
    }
    public Vector2d getBestSamplePos(Vector2d pos) {
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
        Vector2d samplePos = inputSample.getSamplePosition();
        telemetry.addData("x: ", samplePos.x);
        telemetry.addData("y: ", samplePos.y);
        telemetry.addLine();

        telemetry.addData("Point reference: ", closeSample.lowest);
        telemetry.update();
    }
    public void configureWebcam(SampleColor sampleColor) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 5);
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
        if (!withAprilTag) {
            pipeline = new DetectSamples(telemetry, webcam, drive, sampleColor);
            detectSamples = new DetectSamples(telemetry, webcam, drive, sampleColor);
            // setting the pipeline to be only samples detector
        }
        else {
            pipeline = new AprilTagSamplesPipeline(new Apriltag(hardwareMap, drive).getAprilTagAprocessor(), telemetry, drive, sampleColor);
            aprilTagSamplesPipeline = new AprilTagSamplesPipeline(new Apriltag(hardwareMap, drive).getAprilTagAprocessor(), telemetry, drive, sampleColor);
             // setting the pipeline to be both apriltag and samples
        }
        webcam.setPipeline(pipeline);
    }

    public OpenCvPipeline getPipeline() {
        return this.pipeline;
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