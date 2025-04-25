package org.firstinspires.ftc.teamcode.utils.debugging;


import com.jcraft.jsch.ChannelSftp;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;
import com.jcraft.jsch.SftpException;

public class ConceptDatalogger2 {
    private final String RemoteHost = "192.168.1.158";
    private final String Username = "you aint getting my username lol";
    private final String Password = "or password";
    //move all strings to the config.java file

    private ChannelSftp setupJsch() throws JSchException {
        JSch jsch = new JSch();
        jsch.setKnownHosts("~/.ssh/known_hosts");
        Session jschSession = jsch.getSession(Username, RemoteHost);
        jschSession.setPassword(Password);
        jschSession.connect();
        return (ChannelSftp) jschSession.openChannel("sftp");
    }
    public void UploadLog() throws JSchException, SftpException {
        ChannelSftp channelSftp = setupJsch();
        channelSftp.connect();

        String LocalFile = "/home/opi3636/Videos/true.gif";
        String RemoteLocation = "/home/username/";

        channelSftp.put(LocalFile, RemoteLocation + "true.gif");
        channelSftp.disconnect();
    }
}
