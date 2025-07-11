package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
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

//        myBot.runAction(customDriveShim.actionBuilder(new Pose2d(-58.5, -55.5, Math.toRadians(60)), false)
//                .waitSeconds(0.5)
//                .strafeTo(new Vector2d(-37, -18), null, new ProfileAccelConstraint(-100, 200))
//                .splineTo(new Vector2d(-30, -10), Math.toRadians(10), null, new ProfileAccelConstraint(-100, 200))
//                .strafeToLinearHeading(new Vector2d(-58.5, -55.5), Math.toRadians(60), null, new ProfileAccelConstraint(-80, 200))
//                .build()
//        );

        myBot.runAction(customDriveShim.actionBuilder(new Pose2d(-58.5, -55.5, Math.toRadians(60)), false)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-30, -10), null, new ProfileAccelConstraint(-100, 200))
                .strafeTo(new Vector2d(-58.5, -55.5), null, new ProfileAccelConstraint(-80, 200))
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }

    public static CustomDriveShim convertToCustomDriveShim(DriveShim driveShim, Constraints constraints) {
        DriveTrainType driveTrainType = DriveTrainType.MECANUM;
        Pose2d poseEstimate = driveShim.getPoseEstimate();

        return new CustomDriveShim(driveTrainType, constraints, poseEstimate);
    }
}