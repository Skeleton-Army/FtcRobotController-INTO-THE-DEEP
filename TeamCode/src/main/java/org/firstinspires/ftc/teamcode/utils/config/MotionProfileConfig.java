package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotionProfileConfig {
    public static double AXIAL_MULTIPLIER = 1.0;
    public static double LATERAL_MULTIPLIER = 1.0;
    public static double YAW_MULTIPLIER = 1.0;

    public static double MAX_SLEW_RATE = 1.5;
    public static double EXPONENTIAL_SMOOTHING_ALPHA = 0.2; // Adjust between 0 (slow) and 1 (fast)
}
