package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConfig {
    public static String motorName = "intake";
    public static String clawName = "claw";
    public static String wristName = "wrist";

    public static double motorPower = 1;
    public static int extendPosition = -1700;
    public static int retractPosition = -20;

    public static int velocityThreshold = 1200;
    public static double startThreshold = 0.5;

    public static double clawClosed = 0.7;
    public static double clawOpen = 0.47;

    public static double wristExtend = 0.95;
    public static double wristRetract = 0.42;
    public static double wristMiddle = 0.65;

    public static double manualSpeed = 0.6;
}