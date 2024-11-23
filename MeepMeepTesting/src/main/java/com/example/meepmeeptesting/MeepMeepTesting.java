package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

        myBot.runAction(customDriveShim.actionBuilder(new Pose2d(10, -61.5, Math.toRadians(90.00)), false)
                .splineTo(new Vector2d(10, -35), Math.PI / 2, null, new ProfileAccelConstraint(-1000, 100))
                .splineToSplineHeading(new Pose2d(17.07, -49.05, Math.toRadians(90)), Math.toRadians(-14.83))
                .splineToLinearHeading(new Pose2d(41.94, -39.98, Math.toRadians(60)), Math.toRadians(67.62))
                .splineTo(new Vector2d(46.07, -45.68), Math.toRadians(-70.35))
                .build()
        );

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