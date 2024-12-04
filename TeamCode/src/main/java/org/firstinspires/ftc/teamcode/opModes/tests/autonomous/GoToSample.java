package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;


/*
    A test that will go to the nearest sample once, and end the opmode
    This test assumes that the robot is located in the origin (0,0)!

 */

@Autonomous(name = "GoToSample", group = "SA_FTC")
public class GoToSample extends OpMode {

    OpenCvWebcam webcam;
    MecanumDrive drive;
    DetectSamples detectSamples;
    Sample closeSample;

    Pose2d startingPos = new Pose2d(0,0,0); // TOOD: if desired, change it to the actual pos of the robot that stems from the detected sample

    private Sample calculateClosest() {
        // searching for the min value of distance
        List<Sample> samples = detectSamples.samples;
        Sample closest = samples.get(0);

        for (Sample theYellowThingy : detectSamples.samples) {
            if (closest.getDistance() > theYellowThingy.getDistance()) {
                closest = theYellowThingy;
            }
        }

        telemetry.addData("closest: ", closest.getDistance());

        return closest;
    }

    private Vector2d fieldPosition(Sample sample) {
        Pose2d robotPose = drive.pose;
        double sampleAngle = sample.getHorizontalAngle();

        double x = robotPose.position.x + sample.getSampleY() * Math.cos(robotPose.heading.toDouble()) - sample.getSampleX() * Math.sin(robotPose.heading.toDouble());
        double y = robotPose.position.y + sample.getSampleY() * Math.sin(robotPose.heading.toDouble()) + sample.getSampleX() * Math.cos(robotPose.heading.toDouble());
        return new Vector2d(x,-y);
    }

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detectSamples = new DetectSamples(telemetry, webcam);

        webcam.setPipeline(detectSamples);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //webcam.startStreaming(320, 240);

        drive = new MecanumDrive(hardwareMap, startingPos);
    }

    @Override
    public void init_loop() {
        try {
            closeSample = calculateClosest();
            telemetry.addData("x: ", fieldPosition(closeSample).x);
            telemetry.addData("y: ", fieldPosition(closeSample).y);
            telemetry.addLine();

            telemetry.addData("Point reference: ", closeSample.reference);

            telemetry.addLine();
            telemetry.addData("relative x: ", closeSample.getSampleX());
            telemetry.addData("relative y: ", closeSample.getSampleY());
        }
        catch (Exception e) {
            telemetry.addLine("BASA YOSI");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        webcam.stopStreaming();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .splineToConstantHeading(fieldPosition(closeSample) , 0)
                        .waitSeconds(5)
                        .splineToConstantHeading(new Vector2d(0,0) , 0)
                        .build()

        );
    }
    @Override
    public void loop() {

    }
}
