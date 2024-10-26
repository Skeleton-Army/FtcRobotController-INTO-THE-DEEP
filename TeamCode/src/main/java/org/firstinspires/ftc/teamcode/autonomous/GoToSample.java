package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.opencv.Sample;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
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
    SampleMecanumDrive drive;
    DetectSamples detectSamples;
    Sample closeSample;
    Trajectory sampleTrajectory;

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
    public Pose2d sampleFieldPosition(Sample sample) {
        Pose2d robotPose = drive.getPoseEstimate();
        double x = robotPose.getX() + sample.getSampleY() * Math.cos(robotPose.getHeading()) - sample.getSampleX() * Math.sin(robotPose.getHeading());
        double y = robotPose.getY() + sample.getSampleY() * Math.sin(robotPose.getHeading()) + sample.getSampleX() * Math.cos(robotPose.getHeading());
        return new Pose2d(x, y, robotPose.getHeading() + Math.toRadians(sample.getHorizontalAngle()));
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

        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void init_loop() {
        try {
            closeSample = calculateClosest();
        }
        catch (Exception e) {
            telemetry.addLine("BASA YOSI");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

        sampleTrajectory = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(closeSample.getSampleY() - 5, -closeSample.getSampleX()))
                .build();

        drive.followTrajectory(sampleTrajectory);
    }
    @Override
    public void loop() {

    }
}
