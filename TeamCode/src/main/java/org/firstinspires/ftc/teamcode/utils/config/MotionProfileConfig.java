package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotionProfileConfig {
    public static double AXIAL_MULTIPLIER = 1.0;
    public static double LATERAL_MULTIPLIER = 1.0;
    public static double YAW_MULTIPLIER = 1.0;

    public static double SLOW_MODE_MULTIPLIER = 0.3;

    // Larger value - less smoothing
    public static double PARABOLIC_SMOOTHING_BETA = 0.5;
    public static double PARABOLIC_SMOOTHING_POWER = 0;
}
