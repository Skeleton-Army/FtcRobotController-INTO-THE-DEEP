package org.firstinspires.ftc.teamcode.utils.debugging;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
@TeleOp(name = "FtpTest", group = "DataLogging")
public class FtpTest extends LinearOpMode {
    void CreateTestFile(){
        File TestFile = new File("/sdcard/FIRST/datalogs/test.txt");
        try {
            FileWriter WriteToFile = new FileWriter(TestFile.getAbsolutePath());
            WriteToFile.write("14a09a541b95feabdc52f218aaef9579e65ca8973bd29b8b62eab044cc0661aa");
            WriteToFile.close();
            telemetry.addLine("File Written Successfully :)");
            telemetry.update();
        } catch (IOException e) {
            telemetry.addLine("File Write Failed :(");
            telemetry.addLine("Exception Message: " + e.getMessage());
            telemetry.addLine("Stack Trace: " + Arrays.toString(e.getStackTrace()));
            telemetry.update();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            FtpLogging ftpLogging = new FtpLogging();
            CreateTestFile();
            ftpLogging.UploadFile("/sdcard/FIRST/datalogs/test.txt");
            ftpLogging.Disconnect();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
