/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

Credit to @Windwoes (https://github.com/Windwoes).
*/

package org.firstinspires.ftc.teamcode.utils.debugging;

import android.os.Environment;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.config.FtpConfig;

import java.io.File;
import java.io.IOException;
import java.net.ConnectException;
import java.util.List;

@TeleOp(name = "Concept Datalogger v01", group = "Datalogging")
public class ConceptDatalogger extends LinearOpMode {
    Datalog datalog;
    IMU imu;
    VoltageSensor battery;
    DcMotorEx motor;
    String SDcard = Environment.getExternalStorageDirectory().getAbsolutePath();

    @Override
    public void runOpMode() throws InterruptedException {
        battery = hardwareMap.voltageSensor.get("Control Hub");
        imu = hardwareMap.get(IMU.class, "imu");

        // IMU initialization
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.setMsTransmissionInterval(50);

        // File setup
        String LogDir = SDcard + "/FIRST/Datalogs";
        File LogFile = new File(LogDir + "/Log.csv");
        telemetry.addLine("Log file name: " + LogFile.getAbsolutePath());
        telemetry.update();

        datalog = new Datalog(LogFile.getName());

        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

        waitForStart();

        datalog.opModeStatus.set("RUNNING");

        for (int i = 0; opModeIsActive(); i++) {
            datalog.loopCounter.set(i);
            datalog.battery.set(battery.getVoltage());

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            datalog.yaw.set(orientation.getYaw());
            datalog.pitch.set(orientation.getPitch());
            datalog.roll.set(orientation.getRoll());

            datalog.writeLine();

            telemetry.addData("Yaw", datalog.yaw);
            telemetry.addData("Pitch", datalog.pitch);
            telemetry.addData("Roll", datalog.roll);
            telemetry.addLine();
            telemetry.addData("OpMode Status", datalog.opModeStatus);
            telemetry.addData("Loop Counter", datalog.loopCounter);
            telemetry.addData("Battery", datalog.battery);

            for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
                telemetry.addData("Voltage Sensor Name", sensor.getDeviceName());
                telemetry.addData("Voltage", sensor.getVoltage());
            }
            telemetry.update();

            sleep(20);
        }

        if (isStopRequested()) {
            try {
                telemetry.addData("Attempting to connect to ", FtpConfig.Server + ":" + FtpConfig.ServerPort);
                telemetry.update();
                FtpUploading ftp = new FtpUploading();
                if (ftp.IsConnected()){
                    telemetry.addLine("Connetion successful!");
                    List<String> FtpFileList = ftp.ListFiles();
                    int index = 1;
                    String NewLogFileName = "";
                    do {
                        NewLogFileName = "my_log" + index + ".csv";
                        index++;
                    } while(FtpFileList.contains(NewLogFileName));
                    sleep(200);
                    renameFile(LogFile, NewLogFileName);
                    telemetry.addData("Attempting to upload file ", LogFile.getName());
                    telemetry.update();
                    ftp.UploadFile(LogFile, "/" + LogFile.getName(), FtpUploading.ASCII, false);
                    if (ftp.GetReplyCode() == 226) {
                        telemetry.addLine("Upload successful!");
                        ftp.Disconnect();
                        telemetry.addLine("Attempting to delete local file");
                        telemetry.update();
                        if (!LogFile.delete()){
                            telemetry.addLine("File deletion failed! Please remove file later");
                        }
                    } else {
                        throw new IOException("Upload Failed!");
                    }
                } else {
                    telemetry.addLine("Connction Failed! Exiting...");
                    telemetry.update();
                    throw new ConnectException("Connetion to server failed");
                }
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static class Datalog {
        private final Datalogger datalogger;
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField yaw          = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField pitch        = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField roll         = new Datalogger.GenericField("Roll");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");

        public Datalog(String name) {
            datalogger = Datalogger.builder()
                    .setFilename(name)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    .setFields(
                            opModeStatus,
                            loopCounter,
                            yaw,
                            pitch,
                            roll,
                            battery
                    )
                    .build();
        }
        public void writeLine() {
            datalogger.writeLine();
        }
    }
    /**
     * Renames the given file to the new name within the same directory.
     *
     * @param originalFile The original File object.
     * @param newName The new name for the file (without directory path).
     *
     */
    public static void renameFile(File originalFile, String newName) throws IOException {
        if (originalFile == null || newName == null || newName.trim().isEmpty()) {
            throw new IOException("Invalid file or new name.");
        }

        if (!originalFile.exists()) {
            throw new IOException("Original file does not exist: " + originalFile.getAbsolutePath());
        }

        File parentDir = originalFile.getParentFile();
        File renamedFile = new File(parentDir, newName);

        if (renamedFile.exists()) {
            throw new IOException("A file with the new name already exists: " + renamedFile.getAbsolutePath());
        }

        boolean success = originalFile.renameTo(renamedFile);
        if (!success) {
            throw new IOException("Failed to rename the file.");
        }
    }
}