package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SpecimenArmConfig {
    public static String motorName = "specimenArm";
    public static String servoName = "specimenGrip";

    public static double motorPower = 0.2;
    public static int intakePosition = -40;
    public static int outtakePosition = -280;

    public static double gripIntake = 0;
    public static double gripOuttake = 0.1;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static double tP = 0;
    public static int power = 1;
    public static int target = -150;
}
