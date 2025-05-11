package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Outtake;
import org.firstinspires.ftc.teamcode.utils.config.OuttakeConfig;

// very interesting path to push pedro to its limits with the bezier curves it provides

@Autonomous(group = "test", name = "BezierCurveBasket")
public class JuicyBezier extends OpMode {
    Follower follower;

    Pose beginPose = new Pose(24, 24, Math.toRadians(0));
    Outtake outtake;

    public Action depositAction;


    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(beginPose);

        outtake = new Outtake(hardwareMap);
        follower.followPath(GeneratedPath.paths);

        depositAction  = new SequentialAction(
                new ParallelAction(
                        outtake.extend(OuttakeConfig.extendPosition),
                        outtake.bucketReady()
                ),
                outtake.dunk(),
                new ParallelAction(
                        outtake.retract(),
                        outtake.bucketToPosition(OuttakeConfig.bucketHold)
                )
        );
    }

    @Override
    public void loop() {
        follower.update();
       // Pose ExtendingPos = new Pose(ExtendConfig.ExtendX, ExtendConfig.ExtendY, ExtendConfig.ExtendHeading);

        //Pose robotPos = follower.getPose().getAsPedroCoordinates();

        /*boolean Condition = robotPos.getX() >= ExtendingPos.getX() &&
                robotPos.getY() >= ExtendingPos.getY() &&
                robotPos.getHeading() <= ExtendingPos.getHeading();

        Action depositAction = new SequentialAction(
                new ParallelAction(
                        outtake.extend(OuttakeConfig.extendPosition),
                        outtake.bucketReady()
                ),
                outtake.dunk(),
                new ParallelAction(
                        outtake.retract(),
                        outtake.bucketToPosition(OuttakeConfig.bucketHold)
                )
        );*/

        if (follower.atParametricEnd()) {
            follower.followPath(GeneratedPath.paths);
        }
        //follower.followPath(GeneratedPath.paths); // oh, this better be good...
        /*Actions.runBlocking(new ParallelAction(
                new FollowPath(follower, GeneratedPath.paths),
                new ConditionAction(depositAction, () -> Condition)

        )); */

    }

    public static class GeneratedPath {

        public static PathBuilder builder = new PathBuilder();

        public static PathChain paths = builder
                .addPath(
                        new BezierCurve(
                                new Point(24, 24, Point.CARTESIAN),
                                new Point(45.09, 113.831, Point.CARTESIAN),
                                new Point(50.07, 107, Point.CARTESIAN),
                                new Point(-40, 210, Point.CARTESIAN),
                                new Point(41.676, 33.5, Point.CARTESIAN),
                                new Point(24.88, 139.95, Point.CARTESIAN),
                                new Point(71.752, 101.346, Point.CARTESIAN)
                        )
                )
                //.addParametricCallback(0.3, () -> new JuicyBezier().depositAction.run(new TelemetryPacket())) // runs the action when t=0.5
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(275))
                .build();
    }

    @Config
    public static class ExtendConfig {
        public static int ExtendX = 34;
        public static int ExtendY = 102;
        public static double ExtendHeading = Math.toRadians(-18);
    }
}
