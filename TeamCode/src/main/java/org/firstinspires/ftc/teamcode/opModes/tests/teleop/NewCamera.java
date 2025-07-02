package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.config.cameras.CamerasManager;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "new camera", group = "test")
public class NewCamera extends LinearOpMode {
        VisionPortal portal1;
        VisionPortal portal2;
        VisionPortal portal3;
        VisionPortal portal4;

        AprilTagProcessor aprilTagProcessor1;
        AprilTagProcessor aprilTagProcessor2;

        @Override
        public void runOpMode() throws InterruptedException
        {
            // Because we want to show two camera feeds simultaneously, we need to inform
            // the SDK that we want it to split the camera monitor area into two smaller
            // areas for us. It will then give us View IDs which we can pass to the individual
            // vision portals to allow them to properly hook into the UI in tandem.
            int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

            // We extract the two view IDs from the array to make our lives a little easier later.
            // NB: the array is 2 long because we asked for 2 portals up above.
            int portal1ViewId = viewIds[0];
            int portal2ViewId = viewIds[1];
            //int portal3ViewId = viewIds[2];
            //int portal4ViewId = viewIds[3];

            // If we want to run AprilTag detection on two portals simultaneously,
            // we need to create two distinct instances of the AprilTag processor,
            // one for each portal. If you want to see more detail about different
            // options that you have when creating these processors, go check out
            // the ConceptAprilTag OpMode.
            //aprilTagProcessor1 = AprilTagProcessor.easyCreateWithDefaults();
            //aprilTagProcessor2 = AprilTagProcessor.easyCreateWithDefaults();

            // Now we build both portals. The CRITICAL thing to notice here is the call to
            // setLiveViewContainerId(), where we pass in the IDs we received earlier from
            // makeMultiPortalView().

            portal1 = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setLiveViewContainerId(portal1ViewId)
                    .setCameraResolution(new Size(CamerasManager.getByName("Webcam 1").width, CamerasManager.getByName("Webcam 1").height))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)

                    //      .addProcessor(aprilTagProcessor1)
                    .build();


            portal2 = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                    .setLiveViewContainerId(portal2ViewId)
                    .setCameraResolution(new Size(CamerasManager.getByName("Webcam 2").width, CamerasManager.getByName("Webcam 2").height))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                //    .addProcessor(aprilTagProcessor2)
                    .build();

/*
            portal3 = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 3"))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setLiveViewContainerId(portal3ViewId)

                    //    .addProcessor(aprilTagProcessor2)
                    .build();

            portal4 = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 4"))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setLiveViewContainerId(portal4ViewId)
                    //    .addProcessor(aprilTagProcessor2)
                    .build();
*/

            waitForStart();

            // Main Loop
            while (opModeIsActive())
            {
                // Just show some basic telemetry to demonstrate both processors are working in parallel
                // on their respective cameras. If you want to see more detail about the information you
                // can get back from the processor, you should look at ConceptAprilTag.
                telemetry.addData("Number of tags in Camera 1", aprilTagProcessor1.getDetections().size());
                telemetry.addData("Number of tags in Camera 2", aprilTagProcessor2.getDetections().size());
                telemetry.update();
                //sleep(20);
            }
        }
}
