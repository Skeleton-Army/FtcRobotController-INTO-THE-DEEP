package org.firstinspires.ftc.teamcode.utils.actionClasses;

import static org.firstinspires.ftc.teamcode.utils.config.IntakeSensorConfig.colorThresholdHigh;
import static org.firstinspires.ftc.teamcode.utils.config.IntakeSensorConfig.colorThresholdLow;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.utils.config.IntakeSensorConfig;

public class IntakeSensor {
    private final NormalizedColorSensor sensor;

    public IntakeSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, IntakeSensorConfig.name);
    }

    public NormalizedRGBA rgb() {
        NormalizedRGBA rgb = sensor.getNormalizedColors();
        rgb.red = (int) (rgb.red * 255);
        rgb.green = (int) (rgb.green * 255);
        rgb.blue = (int) (rgb.blue * 255);

        return rgb;
    }

    public boolean gotYellowSample() {
        NormalizedRGBA rgb = rgb();
        return rgb.red > colorThresholdHigh && rgb.green > colorThresholdHigh && rgb.blue < colorThresholdLow;
    }

    public boolean gotRedSample() {
        NormalizedRGBA rgb = rgb();
        return rgb.red > colorThresholdHigh && rgb.green < colorThresholdLow && rgb.blue < colorThresholdLow;
    }

    public boolean gotBlueSample() {
        NormalizedRGBA rgb = rgb();
        return rgb.red < colorThresholdLow && rgb.green < colorThresholdLow && rgb.blue > colorThresholdHigh;
    }

    public boolean gotSample() {
        return gotYellowSample() || gotRedSample() || gotBlueSample();
    }
}
