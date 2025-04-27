package org.firstinspires.ftc.teamcode.utils.debugging;

import com.jcraft.jsch.ChannelSftp;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;
import com.jcraft.jsch.SftpException;
import org.firstinspires.ftc.teamcode.utils.config.SSHConfig;

import java.io.File;

public class ConceptDatalogger2 {

    private ChannelSftp setupJsch() throws JSchException {
        JSch jsch = new JSch();
        jsch.setKnownHosts("~/.shh/known_hosts");
        Session jschSession = jsch.getSession(SSHConfig.Username, SSHConfig.RemoteHost);
        jschSession.setPassword(SSHConfig.Password);
        jschSession.connect();
        return (ChannelSftp) jschSession.openChannel("sftp");
    }
    public void UploadLog(String LocalFile, String RemotePath, String RemoteFileName) throws JSchException, SftpException {
        ChannelSftp channelSftp = setupJsch();
        channelSftp.connect();

        String RemoteFile;
        if (RemotePath.endsWith(File.separator)) {
            RemoteFile = RemotePath + RemoteFileName;
        } else {
            RemoteFile = RemotePath + File.separator + RemoteFileName;
        }

        channelSftp.put(LocalFile, RemoteFile);
        channelSftp.disconnect();
    }
}
