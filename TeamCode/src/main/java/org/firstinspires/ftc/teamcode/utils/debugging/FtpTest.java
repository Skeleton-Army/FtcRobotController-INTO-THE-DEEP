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
    String LogDir = SDCard + "/FIRST/Datalogs";
    File TestFile = new File( LogDir + "/test.txt");
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
        telemetry.update();
        try {
            FtpUploading ftpUploading = new FtpUploading();
            ftpUploading.UploadFile(CreateTestFile(), "/test.txt", FtpUploading.ASCII, true);
            telemetry.addData("FTP Reply: ",ftpUploading.GetReplyString() + ftpUploading.GetReplyCode());
            telemetry.addData("Deleting", TestFile.getAbsolutePath());
            telemetry.update();
            if (TestFile.delete()) {
                telemetry.addLine("Deletion failed");
            } else {
                telemetry.addLine("Deletion successful");
            }
            telemetry.addData("Attempting to download ASCII file ", "/AsciiTest.txt");
            telemetry.update();
            File AsciiFile = ftpUploading.DownloadFile("/AsciiTest.txt", new File(SDCard + "/AsciiTest.txt"), FtpUploading.ASCII, false);
            telemetry.addData("FTP reply", ftpUploading.GetReplyString() + ftpUploading.GetReplyCode());
            telemetry.update();
            if (!AsciiFile.exists()) {
                throw new IOException("File failed to download");
            }
            telemetry.addLine("Attempting to downlaod Binary file");
            telemetry.update();
            File BinaryTest = ftpUploading.DownloadFile("/BinaryTest.jpg", new File(SDCard + "/BinaryTest.jpg"), FtpUploading.BINARY, false);
            telemetry.addData("FTP reply", ftpUploading.GetReplyString() + ftpUploading.GetReplyCode());
            telemetry.update();
            if (!BinaryTest.exists()) {
                telemetry.addLine("Download Failed");
                telemetry.update();
            }
            ftpUploading.disconnect();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
