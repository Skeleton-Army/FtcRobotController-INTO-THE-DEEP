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

import java.util.List;

public class Utilities {
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

    public static void OpenCamera(OpenCvWebcam webcamOpencv) {
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
}
