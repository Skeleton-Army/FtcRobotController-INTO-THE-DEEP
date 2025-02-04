package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SpecimenArmConfig {
    public static String motorName = "specimenArm";
    public static String servoName = "specimenGrip";
    public static String grabServoName = "specimenGrab";

    public static int intakePosition = -10;
    public static int middlePosition = -80;
    public static int outtakePosition = -265;
    public static int disabledPosition = 0;

    public static double gripIntake = 1;
    public static double gripOuttake = 0.3;

    public static double grabClose = 0.5;
    public static double grabOpen = 0.8;

    public static double p = 0.015, i = 0.01, d = 0, f = -0.5;
    public static double power = 0.5;

    public static double ticks_in_degree = 134 / 90.0;
}
