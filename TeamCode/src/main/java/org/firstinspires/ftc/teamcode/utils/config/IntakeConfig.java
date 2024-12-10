package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConfig {
    public static String motorName = "intake";
    public static String clawName = "claw";
    public static String wristName = "wrist";

    public static double motorPower = 1;
    public static int extendPosition = -2100;
    public static int retractPosition = -20;

    public static double clawClosed = 0;
    public static double clawOpen = 0.1;

    public static double wristExtend = 1;
    public static double wristRetract = 0.4;
    public static double wristMiddle = 0.6;
}