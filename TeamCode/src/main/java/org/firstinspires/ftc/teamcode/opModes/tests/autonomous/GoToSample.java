/*
package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

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
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


*/
/*
    A test that will go to the nearest sample once, and end the opmode
    This test assumes that the robot is located in the origin (0,0)!

 *//*


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
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    TelemetryPacket packet = new TelemetryPacket();

    private Vector2d fieldPosition(Sample sample) {
        Pose2d robotPose = drive.pose;

        double x = robotPose.position.x + sample.getSampleY() * Math.cos(robotPose.heading.toDouble()) - sample.getSampleX() * Math.sin(robotPose.heading.toDouble());
        double y = robotPose.position.y + sample.getSampleY() * Math.sin(robotPose.heading.toDouble()) + sample.getSampleX() * Math.cos(robotPose.heading.toDouble());
        return new Vector2d(x, y);
    }

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        detectSamples = new DetectSamples(telemetry, webcam, SampleColor.YELLOW);

        webcam.setPipeline(detectSamples);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                */
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
                 *//*

                webcam.startStreaming(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                */
/*
                 * This will be called if the camera could not be opened
                 *//*

            }
        });

        //webcam.startStreaming(320, 240);

        drive = new MecanumDrive(hardwareMap, startingPos);
    }

    @Override
    public void init_loop() {
        try {
            closeSample = Utilities.calculateClosest(detectSamples);
            closeSamplePos = fieldPosition(closeSample);
            telemetry.addLine();

            telemetry.addData("x: ", closeSamplePos.x);
            telemetry.addData("y: ", closeSamplePos.y);
            telemetry.addLine();

            telemetry.addData("Point reference: ", closeSample.reference);

            telemetry.addLine();
            telemetry.addData("relative x: ", closeSample.getSampleX());
            telemetry.addData("relative y: ", closeSample.getSampleY());

            packet.field()
                    .fillRect(closeSamplePos.x, closeSamplePos.y, 10, 10)
                    .fillText("closeSample", closeSamplePos.x, closeSamplePos.y - 20, "10px Arial", 0);

            dashboard.sendTelemetryPacket(packet);

            // uncomment these in case the multiTelemetry doesn't show data on the dashboard
            //dashboardTelemetry.addData("x: ", closeSamplePos.x);
            //dashboardTelemetry.addData("y: ", closeSamplePos.y);
        }
        catch (Exception e) {
            telemetry.addLine("BASA YOSI");
        }
        telemetry.update();
        // also this
        //dashboardTelemetry.update();
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
*/
