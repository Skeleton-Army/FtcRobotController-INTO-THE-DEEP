package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConfig {
    public static String motorName = "intake";
    public static String clawName = "claw";
    public static String wristName = "wrist";

    public static double motorPower = 1;
    public static int extendPosition = 500;
    public static int beforeExtendPosition = 400;
    public static int retractPosition = -100;

    public static int velocityThreshold = 500;
    public static double startThreshold = 0.5;

    public static double clawClosed = 0.25;
    public static double clawOpen = 0.045;

    public static double wristExtend = 0.853;
    public static double wristRetract = 0.35;
    public static double wristMiddle = 0.6;
    public static double wristReady = 0.75;

    public static double manualSpeed = -0.35;
}