package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.actions.FollowPath;
import org.firstinspires.ftc.teamcode.utils.autonomous.WebcamCV;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;

@Disabled
@Autonomous(name = "Auto Test", group = "SA_FTC")
public class AutoTest extends OpMode {
    Follower follower;
    Pose beginPose = new Pose(-48.34, 59.33, Math.toRadians(-90));
    WebcamCV camCV = new WebcamCV(hardwareMap, telemetry, follower);

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(beginPose);

        camCV.configureWebcam(new SampleColor[]{SampleColor.YELLOW});
    }

    @Override
    public void init_loop() {
        if (camCV.lookForSamples()) {
            telemetry.addLine("Detected samples");
        }
        else {
            telemetry.addLine("No samples detected");
        }
    }

    @Override
    public void start() {
        Actions.runBlocking(
                new FollowPath(follower,
                        follower.pathBuilder()
                                .addPath(
                                        // Line 1
                                        new BezierCurve(
                                                new Point(12.303, 24.433, Point.CARTESIAN),
                                                new Point(121.646, 12.130, Point.CARTESIAN),
                                                new Point(122.166, 61.516, Point.CARTESIAN)
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .addPath(
                                        // Line 2
                                        new BezierCurve(
                                                new Point(122.166, 61.516, Point.CARTESIAN),
                                                new Point(131.523, 137.762, Point.CARTESIAN),
                                                new Point(43.841, 115.235, Point.CARTESIAN)
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .addPath(
                                        // Line 3
                                        new BezierCurve(
                                                new Point(43.841, 115.235, Point.CARTESIAN),
                                                new Point(8.318, 101.718, Point.CARTESIAN),
                                                new Point(12.477, 24.433, Point.CARTESIAN)
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .build()
                )
        );
    }

    @Override
    public void loop() {

    }
}
