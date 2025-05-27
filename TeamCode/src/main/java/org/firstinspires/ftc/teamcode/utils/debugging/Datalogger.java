package org.firstinspires.ftc.teamcode.utils.debugging;

import android.os.Environment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;

public class Datalogger {
    public enum AutoTimestamp { NONE, DECIMAL_SECONDS }
    static String SDcard = Environment.getExternalStorageDirectory().getAbsolutePath();
        public interface LoggableField {
        String getName();
        String getValue();
    }

    private File logFile;
    private AutoTimestamp timestampMode;
    private List<LoggableField> fields = new ArrayList<>();
    private BufferedWriter writer;
    private long startTime;

    private Datalogger() {}

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private final Datalogger instance = new Datalogger();

        public Builder setFilename(File logFile) {
            instance.logFile = logFile;
            return this;
        }

        public Builder setAutoTimestamp(AutoTimestamp mode) {
            instance.timestampMode = mode;
            return this;
        }

        public Builder setFields(LoggableField... logFields) {
            for (LoggableField field : logFields) {
                instance.fields.add(field);
            }
            return this;
        }

        public Datalogger build() {
            try {
                instance.writer = new BufferedWriter(new FileWriter(instance.logFile));
                instance.startTime = System.nanoTime();

                // Write CSV header
                List<String> headers = new ArrayList<>();
                if (instance.timestampMode != AutoTimestamp.NONE) {
                    headers.add("Timestamp");
                }
                for (LoggableField field : instance.fields) {
                    headers.add(field.getName());
                }
                instance.writer.write(String.join(",", headers));
                instance.writer.newLine();
                instance.writer.flush();
            } catch (IOException e) {
                throw new RuntimeException("Failed to open log file", e);
            }

            return instance;
        }
    }

    public void writeLine() {
        try {
            List<String> values = new ArrayList<>();
            if (timestampMode == AutoTimestamp.DECIMAL_SECONDS) {
                double seconds = (System.nanoTime() - startTime) / 1e9;
                values.add(String.format("%.3f", seconds));
            }

            for (LoggableField field : fields) {
                values.add(field.getValue());
            }

            writer.write(String.join(",", values));
            writer.newLine();
            writer.flush();
        } catch (IOException e) {
            throw new RuntimeException("Failed to write datalog line", e);
        }
    }

    public static class GenericField implements LoggableField {
        private final String name;
        private String value = "";

        public GenericField(String name) {
            this.name = name;
        }

        public void set(double value) {
            this.value = Double.toString(value);
        }

        public void set(String value) {
            this.value = value;
        }
        public void set(Boolean value) {
            this.value = Boolean.toString(value);
        }
        public void set(int value) {
            this.value = Integer.toString(value);
        }

        @Override
        public String getName() {
            return name;
        }

        @Override
        public String getValue() {
            return value;
        }
    }
    public static File setupLogFile(String localName) {
        String logDir = SDcard + "/FIRST/Datalogs";
        if (localName == null) {
            Calendar calendar = Calendar.getInstance();
            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd");
            SimpleDateFormat timeFormat = new SimpleDateFormat("HH-mm-ss");
            return new File(logDir + "/" + "Log-" + dateFormat.format(calendar.getTime()) + "_" + timeFormat.format(calendar.getTime()) + ".csv");
        } else {
            return new File(logDir + "/" + localName + ".csv");
        }
    }
}
