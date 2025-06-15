package org.firstinspires.ftc.teamcode.utils.config;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class SampleConfig {
    public static double MAX_AREA = 13;
    public static double MIN_AREA = 4;

    static public Scalar lowerBlue = new Scalar(105, 110, 0);
    static public Scalar upperBlue = new Scalar(130, 255, 170);
    static public Scalar lowerRed1 = new Scalar(0, 100, 20);
    static public Scalar upperRed1 = new Scalar(10, 255, 255);
    static public Scalar lowerRed2 = new Scalar(160, 100, 20);
    static public Scalar upperRed2 = new Scalar(180, 255, 255);
    static public Scalar lowerYellow = new Scalar(10, 70, 40);
    static public Scalar upperYellow = new Scalar(35, 255, 255);
}
