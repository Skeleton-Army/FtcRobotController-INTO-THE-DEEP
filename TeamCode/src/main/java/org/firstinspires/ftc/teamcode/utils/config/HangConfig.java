package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HangConfig {
    public static String motorName = "hang";

    public static double hangPower = 1;
    public static int extendPosition = -5400;
    public static int middlePosition = -4400;
    public static int retractPosition = 1100;

    public static int velocityThreshold = 100;
    public static double startThreshold = 0.5;
}
