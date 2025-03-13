package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HangConfig {
    public static String motorName = "hang";

    public static double hangPower = 1;
    public static int extendPosition = -4700;
    public static int middlePosition = -3500;
    public static int retractPosition = 1800;

    public static int velocityThreshold = 100;
    public static double startThreshold = 0.5;
}
