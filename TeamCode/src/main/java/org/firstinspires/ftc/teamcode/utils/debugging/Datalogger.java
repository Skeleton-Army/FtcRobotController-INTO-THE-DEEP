package org.firstinspires.ftc.teamcode.utils.debugging;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Datalogger {
    public enum AutoTimestamp { NONE, DECIMAL_SECONDS }

    public interface LoggableField {
        String getName();
        String getValue();
    }

    private String filename;
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

        public Builder setFilename(String name) {
            instance.filename = "/sdcard/FIRST/Datalogs/" + name;
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
                instance.writer = new BufferedWriter(new FileWriter(instance.filename));
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

        @Override
        public String getName() {
            return name;
        }

        @Override
        public String getValue() {
            return value;
        }
    }
}
