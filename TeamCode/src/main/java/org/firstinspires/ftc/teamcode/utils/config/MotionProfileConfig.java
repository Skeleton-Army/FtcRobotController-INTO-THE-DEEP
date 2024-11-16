package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotionProfileConfig {
    public static double AXIAL_MULTIPLIER = 1.0;
    public static double LATERAL_MULTIPLIER = 1.0;
    public static double YAW_MULTIPLIER = 1.0;

    // Larger value - less smoothing
    public static double MAX_SLEW_RATE = 1.5;
    public static double EXPONENTIAL_SMOOTHING_ALPHA = 0.2; // Between 0 - 1
    public static double PARABOLIC_SMOOTHING_BETA = 1;
    public static int PARABOLIC_SMOOTHING_POWER = 1;
}
