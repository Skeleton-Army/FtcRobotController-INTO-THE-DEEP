package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.MultiCameras;

/*
    This test will run the webcams with their according processors
    Webcam 1 - DetectSamples
    Webcam 2 - Apriltags
    Webcam 3 - Apriltags

    View srcpy for live view of the cameras once in init
 */
@TeleOp(name = "MultiCamerasTest", group = "test")
public class MultiCamerasTest extends OpMode {

    Follower follower;

    MultiCameras cameras;
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        cameras = new MultiCameras("Webcam 1", "Webcam 2", hardwareMap, follower, telemetry);
        //cameras = new MultiCameras("Webcam 1", "Webcam 2", "Webcam 3",hardwareMap, follower, telemetry);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {

    }
}
