package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = 0.0029;
        ThreeWheelIMUConstants.strafeTicksToInches = 0.003;
        ThreeWheelIMUConstants.turnTicksToInches = 0.002;
        ThreeWheelIMUConstants.leftY = 4.25;
        ThreeWheelIMUConstants.rightY = -4.25;
        ThreeWheelIMUConstants.strafeX = -6.9;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "rightBack";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
    }
}
