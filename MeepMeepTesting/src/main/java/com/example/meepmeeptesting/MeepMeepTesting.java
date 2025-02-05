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

        myBot.runAction(customDriveShim.actionBuilder(new Pose2d(0, -62.5, Math.toRadians(90)), false)
                .waitSeconds(3)
                .splineTo(new Vector2d(0, -40), Math.PI / 2, null, new ProfileAccelConstraint(-50, 75))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(32, -38, Math.toRadians(50)), 0)
                .splineToLinearHeading(new Pose2d(32, -38, Math.toRadians(-50)), 0)
                .splineToLinearHeading(new Pose2d(45, -48, Math.toRadians(65)), 0)
                .splineToLinearHeading(new Pose2d(45, -48, Math.toRadians(-65)), 0)
                .splineToLinearHeading(new Pose2d(54, -40, Math.toRadians(45)), 0)
                //.splineToLinearHeading(new Pose2d(54, -40, Math.toRadians(-60)), 0)
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