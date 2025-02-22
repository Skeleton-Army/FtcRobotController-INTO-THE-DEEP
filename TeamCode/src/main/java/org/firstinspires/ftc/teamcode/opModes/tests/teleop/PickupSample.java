package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Webcam;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class PickupSample extends TeleopOpMode {
    MecanumDrive drive;

    Intake intake;
    Outtake outtake;

    /*Apriltag apriltag;*/

    Drive driveActions;

    Webcam webcamSequences;

    WebcamCV camCV;
    boolean collected = false;
    Pose2d sample;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        /*apriltag = new Apriltag(hardwareMap, drive);
        apriltag.enableApriltag();*/

        camCV = new WebcamCV(hardwareMap, telemetry, drive);
        camCV.configureWebcam(new SampleColor[]{SampleColor.YELLOW});


        driveActions = new Drive(drive);
        webcamSequences = new Webcam(driveActions, intake, outtake, "red");
    }

    @Override
    public void init_loop() {
        if (camCV.lookForSamples()) {
            //Vector2d sample = camCV.getBestSamplePos(new Vector2d(0,0));
            sample = camCV.getBestSamplePos(new Vector2d(0,-5));
            telemetry.addLine("Detected samples");
            telemetry.addData("X: ", "" + sample.position.x);
            telemetry.addData("Y: ", "" + sample.position.y);
            telemetry.addData("sample heading: ", Math.toDegrees(sample.heading.toDouble()));
            //camCV.getCloseSampleObject(new Vector2d(0,0)).lowest
        }
        else {
            telemetry.addLine("No samples detected");
        }
    }

    @Override
    public void start() {
        //Vector2d sample = camCV.getBestSamplePos(drive.pose.position);
        //Pose2d sample = camCV.getBestOrientation();
//        telemetry.addData("X: ", "" + sample.position.y);
//        telemetry.addData("Y: ", "" + sample.position.y);
        //telemetry.addData("Orientation: ", "" + sample.heading);
        //runAction(webcamSequences.pickupSample(camCV.getBestSamplePos(drive.pose.position, drive.pose)));

    }

    @Override
    public void loop() {
        if (!collected) {
            Actions.runBlocking(
                    webcamSequences.pickupSample(sample.position)

            );
            collected = true;
        }
        runAllActions();
    }
}
