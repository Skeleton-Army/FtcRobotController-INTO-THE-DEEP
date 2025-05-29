package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConfig {
    public static String motorName = "intake";
    public static String clawName = "claw";
    public static String wrist1Name = "wrist1";
    public static String wrist2Name = "wrist2";
    public static String rotationName = "rotation";

    public static double currentLimit = 0;

    public static int extendPosition = -450;
    public static int retractPosition = 0;

    public static double clawClosed = 0.6;
    public static double clawOpen = 0.33;
    public static double extraOpenClaw = 0;

    public static double wristExtend = 1;
    public static double wristRetract = 0.35;
    public static double wristMiddle = 0.6;
    public static double wristReady = 0.75;

    public static double rotationLeft = 0.82;
    public static double rotationForward = 0.46;
    public static double rotationRight = 0;

    public static double manualSpeed = 0.6;
}