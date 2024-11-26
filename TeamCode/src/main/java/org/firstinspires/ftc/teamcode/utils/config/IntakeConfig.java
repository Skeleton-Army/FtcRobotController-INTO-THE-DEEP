package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConfig {
    public static String motorName = "intake";
    public static String clawName = "claw";
    public static String wristFrontName = "wristFront";
    public static String wristBackName = "wristBack";

    public static double motorPower = 1;
    public static int extendPosition = -2700;
    public static int retractPosition = 100;

    public static double clawClosed = 0.8;
    public static double clawOpen = 1;

    public static double wristClosed = 0.2;
    public static double wristOpen = 0;
}