package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SpecimenArmConfig {
    public static String motorName = "specimenArm";
    public static String servoName = "specimenGrip";

    public static double motorPower = 0.5;
    public static int intakePosition = -40;
    public static int outtakePosition = -280;

    public static double gripIntake = 0;
    public static double gripOuttake = 0.1;
}
