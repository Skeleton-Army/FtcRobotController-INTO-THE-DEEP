package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.config.OneDriverTeleopControls;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;

@TeleOp(name = "Teleop App (One Driver)", group = "SA_FTC")
public class OneDriverTeleopApplication extends TeleopApplication {
    @Override
    public void init() {
        super.init();

        controls = new OneDriverTeleopControls(gamepad1, gamepad2);

        movementUtils = new MovementUtils(gamepad1.gamepad(), controls.SLOW_MODE_BUTTON);
    }
}
