package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;


/*
    A test that will go to the nearest sample once, and end the opmode
    This test assumes that the robot is located in the origin (0,0)!

 */

@Disabled
@Autonomous(name = "GoToSample", group = "SA_FTC")
public class GoToSample extends OpMode {

    OpenCvWebcam webcam;
    MecanumDrive drive;
    DetectSamples detectSamples;
    Sample closeSample;

    Vector2d closeSamplePos;

    Pose2d startingPos = new Pose2d(0,0,0); // TOOD: if desired, change it to the actual pos of the robot that stems from the detected sample

    // Telemetry stuff
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry;

    TelemetryPacket packet = new TelemetryPacket();
    private Sample calculateClosest(List<Sample> samples) {
        // searching for the min value of distance
        if (samples.isEmpty())
            return null;
        Sample closest = samples.get(0);

        for (Sample currentSample : samples) {
            if (closest.getDistance() > currentSample.getDistance()) {
                closest = currentSample;
            }
        }

        return closest;

    }

    private Vector2d fieldPosition(Sample inputSample) {
        Pose2d robotPose = drive.pose;

        double x = robotPose.position.x + inputSample.getSampleY() * Math.cos(robotPose.heading.toDouble()) - inputSample.getSampleX() * Math.sin(robotPose.heading.toDouble());
        double y = robotPose.position.y + inputSample.getSampleY() * Math.sin(robotPose.heading.toDouble()) + inputSample.getSampleX() * Math.cos(robotPose.heading.toDouble());
        return new Vector2d(x, y);
    }

    void printSampleData(Sample inputSample, Vector2d pos) {
        telemetry.addLine();

        telemetry.addData("x: ", pos.x);
        telemetry.addData("y: ", pos.y);
        telemetry.addLine();

        telemetry.addData("Point reference: ", closeSample.lowest);

        telemetry.addLine();
        telemetry.addData("relative x: ", inputSample.getSampleX());
        telemetry.addData("relative y: ", inputSample.getSampleY());

        /*
        packet.field()
                .fillRect(pos.x, pos.y, 10, 10)
                .fillText("closeSample", pos.x, pos.y - 20, "10px Arial", 0);

        dashboard.sendTelemetryPacket(packet);

        // uncomment these in case the multiTelemetry doesn't show data on the dashboard
        dashboardTelemetry.addData("x: ", pos.x);
        dashboardTelemetry.addData("y: ", pos.y);

        dashboardTelemetry.update();

         */
    }

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
/*        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());*/
        dashboardTelemetry = telemetry;
        detectSamples = new DetectSamples(telemetry, webcam, SampleColor.YELLOW);

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
                webcam.startStreaming(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2, OpenCvCameraRotation.UPRIGHT);
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
        List<Sample> samples = detectSamples.samples;
        closeSample = calculateClosest(samples);
        if (closeSample != null) {
            closeSamplePos = fieldPosition(closeSample);
            printSampleData(closeSample, closeSamplePos);
        }
        else {
            telemetry.addLine("BASA YOSI");
        }

        telemetry.update();
        // also this
        dashboardTelemetry.update();
    }

    @Override
    public void start() {
        webcam.stopStreaming();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .splineToConstantHeading(fieldPosition(closeSample) , 0)
                        .waitSeconds(5)
                        .splineToConstantHeading(new Vector2d(0,0) , Math.PI)
                        .build()

        );
    }
    @Override
    public void loop() {

    }
}
