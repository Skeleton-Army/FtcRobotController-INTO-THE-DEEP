package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OuttakeConfig {
    public static String motorName = "outtake";
    public static String bucketName = "bucket";

    public static double motorPower = 1;
    public static int extendPosition = -2950;
    public static int retractPosition = 0;

    public static int velocityThreshold = 300;
    public static double startThreshold = 0.5;

    public static double bucketDunk = 0.35;
    public static double bucketHold = 0;
}