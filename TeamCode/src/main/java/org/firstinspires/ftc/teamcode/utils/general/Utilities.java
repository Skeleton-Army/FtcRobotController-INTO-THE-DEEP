package org.firstinspires.ftc.teamcode.utils.general;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.opencv.DetectSamples;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;
import org.firstinspires.ftc.teamcode.utils.opencv.SampleColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashMap;
import java.util.List;

public class Utilities {
    private static final HashMap<String, Boolean> lastButtonStates = new HashMap<>();

    public static Vector2d rotate(Vector2d original, double angle) {
        double rx = (original.x * Math.cos(angle)) - (original.y * Math.sin(angle));
        double ry = (original.x * Math.sin(angle)) + (original.y * Math.cos(angle));

        return new Vector2d(rx, ry);
    }

    public static void setBulkReadsMode(HardwareMap hardwareMap, LynxModule.BulkCachingMode mode) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(mode);
        }
    }

    // opencv stuff here
    public static OpenCvWebcam createWebcam(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        return webcam;
    }

    public static void openCamera(OpenCvWebcam webcamOpencv) {
        webcamOpencv.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcamOpencv.startStreaming(CameraConfig.halfImageWidth * 2, CameraConfig.halfImageHeight * 2, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    /**
     * Generates an ID representing the location the method is called from.
     * The ID will include the class name, method name, and line number for each element in the stack trace.
     *
     * @return A string representing the full stack trace composition.
     */
    public static String generateCallSiteID() {
        // Get the stack trace of the current thread
        StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();

        // StringBuilder is used here to efficiently build the final string
        StringBuilder idBuilder = new StringBuilder();

        // Loop through all stack trace elements and append their details to the ID
        for (int i = 1; i < stackTrace.length; i++) { // Start from 1 to skip the 'getStackTrace' method
            StackTraceElement caller = stackTrace[i];
            idBuilder.append(caller.getClassName())
                    .append("#")
                    .append(caller.getMethodName())
                    .append(":")
                    .append(caller.getLineNumber());

            if (i < stackTrace.length - 1) {
                idBuilder.append(" -> ");
            }
        }

        return idBuilder.toString();
    }

    /**
     * Checks if an input was pressed (i.e., transitioned from false to true).
     *
     * @param input The current state of the input (true if pressed, false otherwise).
     * @return True if the input transitioned from false to true (was just pressed), false otherwise.
     */
    public static boolean isPressed(boolean input) {
        String uniqueKey = generateCallSiteID();

        // Get the previous state (default to false if not tracked yet)
        boolean lastState = Boolean.TRUE.equals(lastButtonStates.getOrDefault(uniqueKey, false));

        // Update the stored state
        lastButtonStates.put(uniqueKey, input);

        // Return true if the last state was false and the current state is true (just pressed)
        return !lastState && input;
    }

    /**
     * Checks if an input was released (i.e., transitioned from true to false).
     *
     * @param input The current state of the input (true if pressed, false otherwise).
     * @return True if the input transitioned from true to false (was just released), false otherwise.
     */
    public static boolean isReleased(boolean input) {
        String uniqueKey = generateCallSiteID();

        // Get the previous state (default to false if not tracked yet)
        boolean lastState = Boolean.TRUE.equals(lastButtonStates.getOrDefault(uniqueKey, false));

        // Update the stored state
        lastButtonStates.put(uniqueKey, input);

        // Return true if the last state was true and the current state is false (just released)
        return lastState && !input;
    }
}
