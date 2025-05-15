package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConfig {
    public static String motorName = "intake";
    public static String clawName = "claw";
    public static String wrist1Name = "wrist1";
    public static String wrist2Name = "wrist2";
    public static String rotationName = "rotation";

    public static double motorPower = 1;
    public static int extendPosition = 500;
    public static int retractPosition = -100;

    public static int velocityThreshold = 500;
    public static double startThreshold = 0.5;

    public static double clawClosed = 0.61;
    public static double clawOpen = 0.45;
    public static double extraOpenClaw = 0;

    public static double wristExtend = 0.88;
    public static double wristRetract = 0.35;
    public static double wristMiddle = 0.6;
    public static double wristReady = 0.7;

    public static double rotationLeft = 0.82;
    public static double rotationForward = 0.46;
    public static double rotationRight = 0;

    public static double manualSpeed = -0.35;
    public static double tickOverInch = -1; //TODO: FIND CORRECT VALUE PLEASE😭😭😭
    public static double offsetFromCenterX = CameraConfig.pickupSampleOffsetX; //TODO: YOU
}