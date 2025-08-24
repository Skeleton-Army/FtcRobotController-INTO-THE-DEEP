package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.Logger;
import org.psilynx.psikit.io.RLOGServer;

@TeleOp(name = "PsiKitTest")
public class PsiKitTest extends OpMode {
    @Override
    public void init() {
        RLOGServer server = new RLOGServer();
        Logger.addDataReceiver(server);
        Logger.recordMetadata("Opmode Name", "PsiKitTest");
        Logger.start();
        Logger.periodicAfterUser(0,0);
    }

    @Override
    public void loop() {
        double beforeUserStart = Logger.getTimestamp();
        Logger.periodicBeforeUser();
        double beforeUserEnd = Logger.getTimestamp();


        Logger.recordOutput("Logger.recordOutput", "test");
        System.out.println("Printed to stdout");

        double afterUserStart = Logger.getTimestamp();
        Logger.periodicAfterUser(afterUserStart - beforeUserEnd, beforeUserEnd - beforeUserStart);
    }
}
