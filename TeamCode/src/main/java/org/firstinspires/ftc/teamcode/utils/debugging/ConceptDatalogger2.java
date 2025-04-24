package org.firstinspires.ftc.teamcode.utils.debugging;


import com.jcraft.jsch.Channel;
import com.jcraft.jsch.ChannelSftp;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

public class ConceptDatalogger2 {
    private final String RemoteHost = "HOST_NAME_HERE";
    private final String Username = "USERNAME_HERE";
    private final String Password = "PASSWORD_HERE";

    private ChannelSftp setupJsch() throws JSchException {
        JSch jsch = new JSch();
        jsch.setKnownHosts("/Users/john/.ssh/known_hosts");
        Session jschSession = jsch.getSession(Username, RemoteHost);
        jschSession.setPassword(Password);
        jschSession.connect();
        return (ChannelSftp) jschSession.openChannel("sftp");
    }
}
