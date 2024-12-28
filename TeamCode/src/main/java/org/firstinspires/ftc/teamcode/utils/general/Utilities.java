package org.firstinspires.ftc.teamcode.utils.general;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Utilities {
    public static Vector2d rotate(Vector2d original, double angle) {
        double rx = (original.x * Math.cos(angle)) - (original.y * Math.sin(angle));
        double ry = (original.x * Math.sin(angle)) + (original.y * Math.cos(angle));

        return new Vector2d(rx, ry);
    }

    public static void setBulkReadsMode(HardwareMap hardwareMap, LynxModule.BulkCachingMode mode) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(mode);
        }
    }
}
