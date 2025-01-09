package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

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

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        /*apriltag = new Apriltag(hardwareMap, drive);
        apriltag.enableApriltag();*/

        camCV = new WebcamCV(hardwareMap, telemetry, drive);
        camCV.configureWebcam(SampleColor.YELLOW);


        driveActions = new Drive(drive);
        webcamSequences = new Webcam(driveActions, intake, outtake, "red");
    }

    @Override
    public void init_loop() {
        if (camCV.lookForSamples()) {
            Vector2d sample = camCV.getBestSamplePos(new Vector2d(0,0), new Pose2d(0,0,0));
            telemetry.addLine("Detected samples");
            telemetry.addData("X: ", "" + sample.x);
            telemetry.addData("Y: ", "" + sample.y);
        }
        else {
            telemetry.addLine("No samples detected");
        }
    }

    @Override
    public void start() {
        Vector2d sample = camCV.getBestSamplePos(drive.pose.position, drive.pose);
        telemetry.addData("X: ", "" + sample.x);
        telemetry.addData("Y: ", "" + sample.y);
        //runAction(webcamSequences.pickupSample(camCV.getBestSamplePos(drive.pose.position, drive.pose)));

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .splineToConstantHeading(new Vector2d(30,0) , 0)
                        .waitSeconds(5)
                        .splineToConstantHeading(new Vector2d(0,0) , Math.PI)
                        .build()

        );
    }

    @Override
    public void loop() {
        runAllActions();
    }
}
