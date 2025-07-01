package org.firstinspires.ftc.teamcode.utils.autoTeleop;


import android.util.Size;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.config.cameras.CamerasManager;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamplesProcessor;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.vision.VisionPortal;

/*
    This class is creating multiple cameras for auto and teleop mode with live streaming
    Assumes the first webcam is the one designed for sample detections, and the other two for apriltags
*/

public class MultiCameras {

    VisionPortal portal1;
    VisionPortal portal2;
    VisionPortal portal3;
    Apriltag apriltagProcessor1; // TODO: this is null!!
    Apriltag apriltagProcessor2;
    Apriltag apriltagProcessor3;

    public MultiCameras(String CameraName1, String CameraName2, String CameraName3, HardwareMap hardwareMap, Follower follower, Telemetry telemetry) {
        int[] viewIds = VisionPortal.makeMultiPortalView(3, VisionPortal.MultiPortalLayout.VERTICAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 3 long because we asked for 3 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];
        int portal3ViewId = viewIds[2];

        apriltagProcessor1 = new Apriltag(hardwareMap, follower, CameraName1); // creating the apriltag processor by the given camera
        DetectSamplesProcessor detectSamplesProcessor = new DetectSamplesProcessor(telemetry, follower, CameraName1, SampleColor.YELLOW, SampleColor.RED);
        portal1 = new VisionPortal.Builder()
                .addProcessors(detectSamplesProcessor/*, apriltagProcessor1.getAprilTagAprocessor()*/) // processor to the vision
                .setCamera(hardwareMap.get(WebcamName.class, CameraName1))
                .setCameraResolution(new Size(CamerasManager.getByName(CameraName1).width, CamerasManager.getByName(CameraName1).height))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .setShowStatsOverlay(true) // the small pink box at the bottom which shows the fps, resolution ect.
                .setLiveViewContainerId(portal1ViewId)
                .build();

        apriltagProcessor2 = new Apriltag(hardwareMap, follower, CameraName2); // creating the apriltag processor by the given camera
        portal2 = new VisionPortal.Builder()
                .addProcessor(apriltagProcessor2.getAprilTagAprocessor())
                .setCamera(hardwareMap.get(WebcamName.class, CameraName2))
                .setCameraResolution(new Size(CamerasManager.getByName(CameraName2).width, CamerasManager.getByName(CameraName2).height))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .setLiveViewContainerId(portal2ViewId)
                .build();

        apriltagProcessor3 = new Apriltag(hardwareMap, follower, CameraName3); // creating the apriltag processor by the given camera
        portal3 = new VisionPortal.Builder()
                .addProcessor(apriltagProcessor3.getAprilTagAprocessor())
                .setCamera(hardwareMap.get(WebcamName.class, CameraName3))
                .setCameraResolution(new Size(CamerasManager.getByName(CameraName3).width, CamerasManager.getByName(CameraName3).height))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .setLiveViewContainerId(portal3ViewId)
                .build();
    }

    public MultiCameras(String CameraName1, String CameraName2, HardwareMap hardwareMap, Follower follower, Telemetry telemetry) {
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];

        Apriltag apriltagProcessor1 = new Apriltag(hardwareMap, follower, CameraName1);
        DetectSamplesProcessor detectSamplesProcessor = new DetectSamplesProcessor(telemetry, follower, CameraName1, SampleColor.YELLOW, SampleColor.RED);
        portal1 = new VisionPortal.Builder()
                .addProcessors(detectSamplesProcessor/*, apriltagProcessor1.getAprilTagAprocessor()*/) // processor to the vision
                .setCamera(hardwareMap.get(WebcamName.class, CameraName1))
                .setCameraResolution(new Size(CamerasManager.getByName(CameraName1).width, CamerasManager.getByName(CameraName1).height))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .setShowStatsOverlay(true) // the small pink box at the bottom which shows the fps, resolution ect.
                .setLiveViewContainerId(portal1ViewId)
                .build();

        Apriltag apriltagProcessor2 = new Apriltag(hardwareMap, follower, CameraName2);
        portal2 = new VisionPortal.Builder()
                .addProcessor(apriltagProcessor2.getAprilTagAprocessor())
                .setCamera(hardwareMap.get(WebcamName.class, CameraName2))
                .setCameraResolution(new Size(CamerasManager.getByName(CameraName2).width, CamerasManager.getByName(CameraName2).height))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .setLiveViewContainerId(portal2ViewId)
                .build();

    }
}
