package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SpecimenArmConfig {
    public static String motorName = "specimenArm";
    public static String servoName = "specimenGrip";
    public static String grabServoName = "specimenGrab";

    public static int intakePosition = -10;
    public static int outtakePosition = -625;

    public static double gripIntake = 0.75;
    public static double gripOuttake = 0.05;

    public static double grabClose = 0.76;
    public static double grabOpen = 0.45;

    public static double p = 0.005, i = 0.08, d = 0.0003;
    public static double f = 0.0004;
    public static int topPos = -440;
}
