package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.wiggleBackDistance;
import static org.firstinspires.ftc.teamcode.utils.config.CameraConfig.wiggleDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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
import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
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

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        camCV = new WebcamCV(hardwareMap, telemetry, drive);
        camCV.configureWebcam(new SampleColor[]{SampleColor.YELLOW, SampleColor.RED});

        driveActions = new Drive(drive, camCV, telemetry);
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
            telemetry.addData("sample center x: ", targetSample.center.x);
            telemetry.addData("sample center y: ", targetSample.center.y);
        }
        else {
            telemetry.addLine("No samples detected");
        }
    }

    @Override
    public void start() {
        Actions.runBlocking(
                driveActions.alignToSample(targetSample.getSamplePosition().position)
        );

        double orientation = -Drive.targetSampleStatic.orientation;
        double normalizedOrientation = (90 - Math.abs(orientation)) * Math.signum(orientation);
        double rotationTarget = normalizedOrientation / 90;

//        double wiggleX = Math.sin(Math.toRadians(normalizedOrientation)) * wiggleDistance;
//        double wiggleY = Math.cos(Math.toRadians(normalizedOrientation)) * wiggleDistance;
//        double wiggleBackX = Math.sin(Math.toRadians(normalizedOrientation)) * wiggleBackDistance;
//        double wiggleBackY = Math.cos(Math.toRadians(normalizedOrientation)) * wiggleBackDistance;

        Actions.runBlocking(
                intake.rotate(rotationTarget)
        );

        Actions.runBlocking(
                new SequentialAction(
                        intake.openClaw(),
                        intake.wristReady(),
                        intake.extend(),
                        intake.extendWrist(),
                        new SleepAction(0.2),

//                        drive.actionBuilder(drive.pose)
//                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x + wiggleX, drive.pose.position.y - wiggleY), null, new ProfileAccelConstraint(-100, 100))
//                                .afterDisp(wiggleDistance, intake.closeClaw())
//                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x - wiggleBackX, drive.pose.position.y + wiggleBackY), null, new ProfileAccelConstraint(-100, 100))
//                                .build(),

                        intake.closeClaw(),
                        new SleepAction(0.2),

                        outtake.hold(),
                        intake.retractWrist(),
                        new SleepAction(0.1),
                        intake.rotate(0),
                        intake.retract(),
                        intake.openClaw(),
                        new SleepAction(0.05),
                        intake.wristMiddle(),
                        new SleepAction(0.2)
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
