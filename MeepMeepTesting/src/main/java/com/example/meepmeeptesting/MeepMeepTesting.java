package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -61.5, Math.toRadians(90.00)))
                .splineTo(new Vector2d(10,-30), Math.toRadians(90.00))
                .splineTo(new Vector2d(10,-40), Math.toRadians(90.00))
                .splineTo(new Vector2d(38,-43), Math.toRadians(45.00))
                .turnTo(Math.toRadians(-45.00))
                .splineTo(new Vector2d(47,-43), Math.toRadians(45.00))
                .turnTo(Math.toRadians(-45.00))
                .splineTo(new Vector2d(57,-43), Math.toRadians(45.00))
                .turnTo(Math.toRadians(-45.00))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}