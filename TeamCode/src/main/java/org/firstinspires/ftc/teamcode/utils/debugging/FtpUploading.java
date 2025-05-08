package org.firstinspires.ftc.teamcode.utils.debugging;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.utils.config.FtpConfig;

import org.apache.commons.net.PrintCommandListener;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPReply;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Utility class for uploading files to an FTP server.
 * <p>
 * Uses Apache Commons Net FTPClient to handle FTP communication.
 */
public class FtpUploading {

    /**
     * ASCII file transfer mode constant.
     */
    public static final int ASCII = 0;

    /**
     * Binary file transfer mode constant.
     */
    public static final int BINARY = 2;

    /**
     * FTP client used for communication with the FTP server.
     */
    private FTPClient ftp;

    /**
     * Constructs a new {@code FtpUploading} instance and connects to the FTP server.
     *
     * @throws IOException if the connection to the server fails.
     */
    public FtpUploading() throws IOException {
        Connect();
    }
    public boolean IsConnected() {
        return ftp.isConnected();
    }
    /**
     * Establishes a connection to the FTP server using configuration from {@link FtpConfig}.
     *
     * @throws IOException if the connection or login fails.
     */
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

    /**
     * Disconnects from the FTP server.
     *
     * @throws IOException if an I/O error occurs while disconnecting.
     */
    public void Disconnect() throws IOException {
        this.ftp.disconnect();
    }

    /**
     * Retrieves the last reply code from the FTP server.
     *
     * @return the reply code.
     */
    public int GetReplyCode() {
        return this.ftp.getReplyCode();
    }

    /**
     * Retrieves the last reply string from the FTP server.
     *
     * @return the reply string.
     */
    public String GetReplyString() {
        return this.ftp.getReplyString();
    }

    /**
     * Uploads a file to the FTP server.
     *
     * @param LocalFilePath  the local file path to upload.
     * @param RemotePath the remote destination path on the FTP server.
     * @param FileType   the file transfer mode (use {@link #ASCII} or {@link #BINARY}).
     * @throws IOException if the file doesn't exist or upload fails.
     */
    public void UploadFile(String LocalFilePath, String RemotePath, int FileType) throws Exception {
        File FileToUpload = new File(LocalFilePath);
        if (FileType == 0 || FileType == 2) {
            this.ftp.setFileType(FileType);
            if (FileToUpload.exists()) {
                this.ftp.storeFile(RemotePath, new FileInputStream(FileToUpload));
            } else {
                throw new IOException("Input file does not exist");
            }
        } else {
            throw new Exception("Filetype isnt set to ASCII or Binary");
        }
   }
    /**
     * Uploads a file to a remote FTP server.
     *
     * <p>This method sets the file type for the FTP client and uploads a local file to the specified remote path.
     * If the local file does not exist, it throws an {@link IOException}.</p>
     *
     * @param LocalFilePath the {@link File} object representing the path to the local file to be uploaded
     * @param RemotePath the destination path on the remote FTP server where the file should be uploaded
     * @param FileType the file type for FTP transmission
     * @throws IOException if the local file does not exist or if an I/O error occurs during upload
     */
    public void UploadFile(@NonNull File LocalFilePath, String RemotePath, int FileType) throws Exception {
        if (FileType == 0 || FileType == 2){
            this.ftp.setFileType(FileType);
            if (LocalFilePath.exists()) {
                this.ftp.storeFile(RemotePath, new FileInputStream(LocalFilePath));
            } else {
                throw new IOException("Input file does not exist");
            }
        }
        else {
            throw new Exception("Filetype isnt set to ASCII or Binary");
        }

    }

}
