package org.firstinspires.ftc.teamcode.utils.debugging;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
public class dateTime extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Get current time and date
        LocalDateTime now = LocalDateTime.now();
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");
        String formattedTime = now.format(formatter);

        telemetry.addLine("OpMode started at: " + formattedTime);
        telemetry.update();

        waitForStart();

        // Your robot code goes here

        while (opModeIsActive()) {
            telemetry.addLine("Running...");
            telemetry.update();
        }
    }
}
