package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.config.IntakeSensorConfig;

public class IntakeSensor {
    private final ColorSensor sensor;

    public IntakeSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(ColorSensor.class, IntakeSensorConfig.name);
    }

    public int red() {
        return sensor.red();
    }

    public int green() {
        return sensor.green();
    }

    public int blue() {
        return sensor.blue();
    }
}
