package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeSensorConfig {
    public static String name = "intakeSensor";

    public static int colorThresholdHigh = 200;
    public static int colorThresholdLow = 100;

    public static int multiplier = 255;
    public static float gain = 0.5f; // Less reduces noise but makes colors dimmer | Makes colors brighter but increases noise
    public static int sampleCount = 5; // Average over X samples
}
