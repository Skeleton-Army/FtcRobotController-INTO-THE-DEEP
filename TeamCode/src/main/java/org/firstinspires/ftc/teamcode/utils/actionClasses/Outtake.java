package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private DcMotorEx motor;

    public Outtake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "<fill this>"); //TODO: find out what the name is
    }

}
