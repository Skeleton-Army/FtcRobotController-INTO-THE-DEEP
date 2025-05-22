package org.firstinspires.ftc.teamcode.utils.customExceptions;

public class ConfigurationException extends Exception{
    public ConfigurationException(String message) {
       super(message);
    }

    public ConfigurationException(String message, Throwable cause) {
        super(message, cause);
    }
}
