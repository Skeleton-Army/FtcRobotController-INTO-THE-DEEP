package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeSensorConfig {
    public static String name = "intakeSensor";

    public static int colorThresholdHigh = 1;
    public static int colorThresholdLow = 0;

    public static int multiplier = 255;
    public static float gain = 0.5f; // Less reduces noise but makes colors dimmer | Makes colors brighter but increases noise
}
