package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConfig {
    public static String motorName = "intake";
    public static String clawName = "claw";
    public static String wristName = "wrist";
    public static String rotationName = "rotation";

    public static double motorPower = 1;
    public static int extendPosition = 500;
    public static int retractPosition = -100;

    public static int velocityThreshold = 500;
    public static double startThreshold = 0.5;

    public static double clawClosed = 0.48;
    public static double clawOpen = 0.25;

    public static double wristExtend = 0.7875;
    public static double wristRetract = 0.3;
    public static double wristMiddle = 0.6;
    public static double wristReady = 0.65;

    public static double rotationLeft = 0.85;
    public static double rotationRight = 0.2;

    public static double manualSpeed = -0.35;
}