package org.firstinspires.ftc.teamcode.utils.debugging;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

@TeleOp(name = "FtpTest", group = "DataLogging")
public class FtpTest extends LinearOpMode {
    String SDCard = Environment.getExternalStorageDirectory().getAbsolutePath();
    File TestFile = new File( SDCard + "/FIRST/Datalogs/test.txt");
    String CreateTestFile(){
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
        return TestFile.getAbsolutePath();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        telemetry.addData("File to upload: ", TestFile.getAbsolutePath());
        try {
            FtpUploading ftpUploading = new FtpUploading();
            ftpUploading.UploadFile(CreateTestFile(), "/test.txt", FtpUploading.ASCII);
            telemetry.addData("FTP Reply: ",ftpUploading.GetReplyString() + ftpUploading.GetReplyCode());
            ftpUploading.Disconnect();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
