package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;

@TeleOp(name = "Teleop App", group = "SA_FTC")
public class TeleopApplication extends OpMode {
    public static TeleopApplication Instance;

    MovementUtils movementUtils;

    @Override
    public void init() {
        Instance = this;

        movementUtils = new MovementUtils(hardwareMap, PoseStorage.currentPose);
    }

    @Override
    public void loop() {
        //movementUtils.movement(gamepad1);
        movementUtils.fieldCentricMovement();

        // Debugging
        telemetry.update();
    }
}