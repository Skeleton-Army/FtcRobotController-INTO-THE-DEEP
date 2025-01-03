package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Webcam;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.general.Utilities;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;
import org.openftc.easyopencv.OpenCvWebcam;

public class PickupSample extends TeleopOpMode {
    MecanumDrive drive;

    Intake intake;
    Outtake outtake;

    Apriltag apriltag;

    Drive driveActions;

    DetectSamples detectSamples;

    Webcam webcamSequences;

    OpenCvWebcam webcamOpencv;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        apriltag = new Apriltag(hardwareMap, drive);
        apriltag.enableApriltag();

        webcamOpencv = Utilities.createWebcam(hardwareMap);
        detectSamples = Utilities.initializeCamera(telemetry, webcamOpencv);
        Utilities.OpenCamera(webcamOpencv);

        driveActions = new Drive(drive, apriltag, detectSamples);
        webcamSequences = new Webcam(driveActions, intake, outtake, "red");
    }

    @Override
    public void init_loop() {
        telemetry.addLine("let it run to start the camera");
        telemetry.update();
    }

    @Override
    public void start() {
        runAction(webcamSequences.pickupSample());
    }

    @Override
    public void loop() {

    }
}
