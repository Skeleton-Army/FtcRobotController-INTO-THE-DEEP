package org.firstinspires.ftc.teamcode.utils.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class WebcamCV {
    public OpenCvWebcam webcam;
    DetectSamples detectSamples;
    List<Sample> samples;
    Sample closeSample;
    Vector2d closeSamplePos;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    MecanumDrive drive;

    public WebcamCV(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.drive = drive;
    }
    private Vector2d fieldPosition(Sample inputSample, Pose2d detectionPose) {
        double x = detectionPose.position.x + inputSample.getSampleY() * Math.cos(detectionPose.heading.toDouble()) - inputSample.getSampleX() * Math.sin(detectionPose.heading.toDouble());
        double y = detectionPose.position.y + inputSample.getSampleY() * Math.sin(detectionPose.heading.toDouble()) + inputSample.getSampleX() * Math.cos(detectionPose.heading.toDouble());
        return new Vector2d(x, y);
    }
    private double distanceFromPosition(Sample currSample, Vector2d pos, Pose2d detectionPose) {
        Vector2d samplePos = fieldPosition(currSample, detectionPose);

        return Math.pow(samplePos.x - pos.x, 2) + Math.pow(samplePos.y - pos.y, 2);
    }
    public Vector2d getBestSamplePos(Vector2d pos, Pose2d detectionPose) {
        // searching for the min value of distance
        if (samples.isEmpty())
            return null;
        Sample closest = samples.get(0);

        for (Sample currSample : samples) {
            if (distanceFromPosition(closest, pos, detectionPose) > distanceFromPosition(currSample, pos, detectionPose)) {
                closest = currSample;
            }
        }

        return fieldPosition(closest, detectionPose);
    }
    private void printSampleData(Sample inputSample, Vector2d pos) {
        telemetry.addLine();

        telemetry.addData("x: ", pos.x);
        telemetry.addData("y: ", pos.y);
        telemetry.addLine();

        telemetry.addData("Point reference: ", closeSample.lowest);

        telemetry.addLine();
        telemetry.addData("relative x: ", inputSample.getSampleX());
        telemetry.addData("relative y: ", inputSample.getSampleY());
    }
    public void configureWebcam(SampleColor sampleColor) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        telemetry.addLine("ooooo");
        FtcDashboard.getInstance().startCameraStream(webcam, 5);
        detectSamples = new DetectSamples(telemetry, webcam, sampleColor);
        webcam.setPipeline(detectSamples);
        telemetry.addLine("nnnnnnn");
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
        telemetry.addLine("yyyyyyyyyyy");
    }
    public boolean lookForSamples() {
        List<Sample> newSamples = detectSamples.samples;

        if (!(newSamples.isEmpty())) {
            samples = newSamples;
        }
        return (samples != null) && (!samples.isEmpty());
    }
}