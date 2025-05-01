package org.firstinspires.ftc.teamcode.utils.debugging;

import org.apache.commons.net.ftp.FTPFile;
import  org.firstinspires.ftc.teamcode.utils.config.FtpConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.net.PrintCommandListener;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPReply;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

@TeleOp(name = "FtpTest", group = "DataLogging")
public class FtpTest extends LinearOpMode {
   private FTPClient ftp;
   void Connect() throws IOException {
        ftp = new FTPClient();

        ftp.addProtocolCommandListener(new PrintCommandListener(new PrintWriter(System.out)));

        ftp.connect(FtpConfig.Server, FtpConfig.ServerPort);
        int reply = ftp.getReplyCode();
        telemetry.addData("Ftp Reply Code", reply);
        telemetry.update();
        if (!FTPReply.isPositiveCompletion(reply)) {
            ftp.disconnect();
            throw new IOException("Exception in connecting to FTP Server");
        }

        ftp.login(FtpConfig.User, FtpConfig.Password);
   }

    void Disconnect() throws IOException {
        ftp.disconnect();
    }
   String[] ListFiles() throws IOException {
       FTPFile[] Files = ftp.listFiles();
       return (String[]) Arrays.stream(Files).toArray();
   }
    void UploadFile() throws IOException {
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
        /*try {
            UploadFile();
        } catch (IOException e) {
            telemetry.addLine(":( Exception caught");
            telemetry.addLine("Exception Message: " + e.getMessage());
            telemetry.update();
        }*/
        try {
            String[] FileList = ListFiles();
            for (int i = 0; i < FileList.length; i++) {
                telemetry.addLine(FileList[i]);
            }
            telemetry.update();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

