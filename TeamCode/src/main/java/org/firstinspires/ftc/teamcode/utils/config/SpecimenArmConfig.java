package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SpecimenArmConfig {
    public static String motorName = "specimenArm";
    public static String servoName = "specimenGrip";
    public static String grabServoName = "specimenGrab";

    public static int intakePosition = -35;
    public static int middlePosition = -80;
    public static int outtakePosition = -270;
    public static int disabledPosition = -5;

    public static double gripIntake = 0.35;
    public static double gripOuttake = 1;

    public static double grabIntake = 0.5;
    public static double grabOuttake = 1;

    public static double p = 0.055, i = 0, d = 0.0001, f = -0.3;
    public static double power = 0.5;

    public static double ticks_in_degree = 134 / 90.0;
}
