package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HangConfig {
    public static String hangMotorName = "hang";

    public static double hangPower = 1;
    public static int hangExtendPosition = 4000;
    public static int hangRetractPosition = 0;

    public static double outtakePower = 0.5;
    public static int outtakeExtendPosition = -2000;
    public static int outtakeRetractPosition = 0;

    public static int velocityThreshold = 200;
    public static double startThreshold = 3;
}
