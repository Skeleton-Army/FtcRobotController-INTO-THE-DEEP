package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Webcam;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;

@Autonomous
public class PickupSample extends TeleopOpMode {
    MecanumDrive drive;

    Intake intake;
    Outtake outtake;

    Drive driveActions;

    Webcam webcamSequences;

    WebcamCV camCV;
    Sample targetSample;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        camCV = new WebcamCV(hardwareMap, telemetry, drive);
        camCV.configureWebcam(new SampleColor[]{SampleColor.YELLOW});

        driveActions = new Drive(drive, camCV);
        webcamSequences = new Webcam(driveActions, intake, outtake, "red");
    }

    @Override
    public void init_loop() {
        if (camCV.lookForSamples()) {
            targetSample = camCV.getBestSample(new Vector2d(30,0));
            Pose2d targetSamplePos = targetSample.getSamplePosition();

            telemetry.addLine("Detected samples");
            telemetry.addData("X: ", "" + targetSamplePos.position.x);
            telemetry.addData("Y: ", "" + targetSamplePos.position.y);
            telemetry.addData("sample heading: ", Math.toDegrees(targetSamplePos.heading.toDouble()));

            WebcamCV.drawSample(camCV.getCloseSampleObject(new Vector2d(0,0)));
        }
        else {
            telemetry.addLine("No samples detected");
        }
    }

    @Override
    public void start() {
        Actions.runBlocking(
                driveActions.alignToSampleContinuous(targetSample)
        );

        Actions.runBlocking(
                new SequentialAction(
                        intake.openClaw(),
                        intake.wristReady(),
                        intake.extend(),
                        intake.extendWrist(),
                        new SleepAction(0.4),
                        intake.closeClaw(),
                        new SleepAction(0.4),
                        intake.retractWrist(),
                        new ParallelAction(
                                intake.retract(),
                                new SleepAction(0.4)
                        ),
                        intake.openClaw(),
                        intake.wristMiddle()
                )
        );

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );

        requestOpModeStop();
    }

    @Override
    public void loop() {}
}
