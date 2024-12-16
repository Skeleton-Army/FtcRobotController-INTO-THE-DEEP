package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SpecimenArmConfig {
    public static String motorName = "specimenArm";
    public static String servoName = "specimenGrip";

    public static double motorPower = 0.5;
    public static int intakePosition = -1000;
    public static int outtakePosition = -500;
    public static int disabledPosition = -20;

    public static double gripIntake = 0;
    public static double gripOuttake = 0.1;
}
