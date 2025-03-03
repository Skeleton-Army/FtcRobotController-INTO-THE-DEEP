package org.firstinspires.ftc.teamcode.utils.actionClasses;

import static org.firstinspires.ftc.teamcode.utils.config.IntakeSensorConfig.colorThresholdHigh;
import static org.firstinspires.ftc.teamcode.utils.config.IntakeSensorConfig.colorThresholdLow;
import static org.firstinspires.ftc.teamcode.utils.config.IntakeSensorConfig.gain;
import static org.firstinspires.ftc.teamcode.utils.config.IntakeSensorConfig.multiplier;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.utils.config.IntakeSensorConfig;

public class IntakeSensor {
    private final NormalizedColorSensor sensor;

    public IntakeSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, IntakeSensorConfig.name);

        sensor.setGain(gain);
    }

    public int[] getRGBValues() {
        NormalizedRGBA rgb = sensor.getNormalizedColors();

        return new int[]{
                (int) (rgb.red * multiplier),
                (int) (rgb.green * multiplier),
                (int) (rgb.blue * multiplier)
        };
    }

    public boolean gotYellowSample() {
        int[] rgb = getRGBValues();
        return rgb[0] >= colorThresholdHigh && rgb[1] >= colorThresholdHigh && rgb[2] <= colorThresholdLow;
    }

    public boolean gotRedSample() {
        int[] rgb = getRGBValues();
        return rgb[0] >= colorThresholdHigh && rgb[1] <= colorThresholdLow && rgb[2] <= colorThresholdLow;
    }

    public boolean gotBlueSample() {
        int[] rgb = getRGBValues();
        return rgb[0] <= colorThresholdLow && rgb[1] <= colorThresholdLow && rgb[2] >= colorThresholdHigh;
    }

    public boolean gotSample() {
        return gotYellowSample() || gotRedSample() || gotBlueSample();
    }
}
