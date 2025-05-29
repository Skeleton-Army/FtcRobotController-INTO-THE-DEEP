package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.config.OneDriverTeleopControls;
import org.firstinspires.ftc.teamcode.utils.config.TeleopControls;
import org.firstinspires.ftc.teamcode.utils.teleop.MovementUtils;

@TeleOp(name = "Teleop App (One Driver)", group = "SA_FTC")
public class OneDriverTeleopApplication extends TeleopApplication {
    private boolean intakeRotated = false;

    @Override
    public void init() {
        super.init();

        controls = new OneDriverTeleopControls(gamepad1, gamepad2);

        movementUtils = new MovementUtils(gamepad1.gamepad(), controls.SLOW_MODE_BUTTON);
    }

    @Override
    public void runIntakeRotation() {
        // Intake claw rotation
        if (isInState("intake", 1)) {
            if (controls.INTAKE_ROTATION.isJustPressed()) {
                intakeRotated = !intakeRotated;

                runAction(intake.rotate(intakeRotated ? 1 : 0));
            }
        }
        else {
            intakeRotated = false;
        }
    }
}
