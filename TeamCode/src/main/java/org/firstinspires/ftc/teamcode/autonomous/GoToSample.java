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
import org.openftc.easyopencv.OpenCvWebcam;


/*
    A test that will go to the nearest sample once, and end the opmode
    This test assumes that the robot is located in the origin (0,0)!

 */

@Autonomous(name = "GoToSample", group = "tests")
public class GoToSample extends OpMode {

    OpenCvWebcam webcam;
    SampleMecanumDrive drive;
    DetectSamples detectSamples;
    Sample closeSample;
    Trajectory sampleTrajectory;

    private Sample calculateClosest() {
        // searching for the min value of distance
        Sample closest = detectSamples.samples.get(0);
        for (Sample theYellowThingy : detectSamples.samples) {
            if (closest.getDistance() > theYellowThingy.getDistance()) {
                closest = theYellowThingy;
            }
        }

        telemetry.addData("closest: ", closest.getDistance());

        return closest;
    }

    @Override
    public void init() {
        detectSamples = new DetectSamples(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(detectSamples);
        webcam.startStreaming(320, 240);

        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void init_loop() {
        closeSample = calculateClosest();
        telemetry.update();
    }

    @Override
    public void start() {
        drive.setPoseEstimate(new Pose2d(0,0,0));

        sampleTrajectory = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(closeSample.getSampleX(), closeSample.getSampleY()), 0)
                .build();

    }
    @Override
    public void loop() {
        drive.followTrajectory(sampleTrajectory);
    }
}
