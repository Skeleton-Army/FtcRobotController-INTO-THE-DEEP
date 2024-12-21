package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.QuinticSpline1d;
import com.acmerobotics.roadrunner.QuinticSpline2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        Constraints constraints = new Constraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(constraints)
                .build();

        CustomDriveShim customDriveShim = convertToCustomDriveShim(myBot.getDrive(), constraints);

        QuinticSpline1d splinex = new QuinticSpline1d(287.9999999999997, -719.9999999999994, 479.9999999999996, 0.0, 0.0, -24.0);
        QuinticSpline1d spliney = new QuinticSpline1d(221.99999999999983, -554.9999999999995, 369.99999999999966, 0.0, 0.0, -37.0);

        QuinticSpline2d  spline = new QuinticSpline2d(splinex, spliney);

        Vector2d v1 = spline.get(1, 1).value();

        myBot.runAction(customDriveShim.actionBuilder(new Pose2d(-24, 50, Math.toRadians(0)), false)
                //.splineTo(new Vector2d(10, -35), Math.PI / 2, null, new ProfileAccelConstraint(-100, 100))
                .splineTo(v1, new Rotation2d(Math.toRadians(-180),Math.toRadians(-25)))
//                .splineToConstantHeading(v1, Math.toRadians(90))
                .build()
        );

        System.out.println(v1);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static CustomDriveShim convertToCustomDriveShim(DriveShim driveShim, Constraints constraints) {
        DriveTrainType driveTrainType = DriveTrainType.MECANUM;
        Pose2d poseEstimate = driveShim.getPoseEstimate();

        return new CustomDriveShim(driveTrainType, constraints, poseEstimate);
    }
}