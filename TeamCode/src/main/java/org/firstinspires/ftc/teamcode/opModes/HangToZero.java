package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.actionClasses.Hang;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;

@TeleOp(name = "Hang To Zero", group = "SA_FTC")
public class HangToZero  extends TeleopOpMode {
    Hang hang;

    @Override
    public void init() {
        hang = new Hang(hardwareMap);
    }

    @Override
    public void start() {
        runAction(hang.hangToPosition(-5000, 0.4, false));
    }

    @Override
    public void loop() {

    }
}
