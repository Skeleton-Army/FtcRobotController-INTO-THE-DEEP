package org.firstinspires.ftc.teamcode.utils.debugging;


import org.firstinspires.ftc.teamcode.utils.config.FtpConfig;

import org.apache.commons.net.PrintCommandListener;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPReply;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

public class FtpLogging{
   private FTPClient ftp;
   public FtpLogging() throws IOException {
       Connect();
   }

    private void Connect() throws IOException {
        this.ftp = new FTPClient();

        this.ftp.addProtocolCommandListener(new PrintCommandListener(new PrintWriter(System.out)));

        this.ftp.connect(FtpConfig.Server, FtpConfig.ServerPort);
        int reply = this.ftp.getReplyCode();
        if (!FTPReply.isPositiveCompletion(reply)) {
            this.ftp.disconnect();
            throw new IOException("Exception in connecting to FTP Server");
        }

        this.ftp.login(FtpConfig.User, FtpConfig.Password);
    }

    public void Disconnect() throws IOException {
        this.ftp.disconnect();
    }

    public void UploadFile(String path) throws IOException {
        File FileToUpload = new File(path);
        if (FileToUpload.exists()) {

        } else {
            throw new IOException("Input file does not exist");
        }
    }
}

