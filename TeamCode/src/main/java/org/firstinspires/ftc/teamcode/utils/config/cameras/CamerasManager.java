package org.firstinspires.ftc.teamcode.utils.config.cameras;

import java.util.*;

public class CamerasManager {

    // Static list of all cameras
    public static final List<Camera> CAMERAS = Arrays.asList(
            new Camera(
                    "Webcam 1",
                    640, 480,
                    // cameraMatrix
                    new double[]{496.040455195, 0, 322.226720938, 0, 496.912794034, 179.36243685, 0, 0, 1},
                    // distortions coefficients
                    new double[]{0.0145220035986483, -0.0121145654176830, 0.0, 0.0, 0.0},
                    6.1, 7.87, 12.5,
                    0.0, 10, 0.0
            ),
            new Camera(
                    "Webcam 2",
                    640, 480,
                    // cameraMatrix
                    new double[]{688.9972, 0, 613.914, 0, 688.5173, 397.1161, 0, 0, 1},
                    // distortions coefficients
                    new double[]{-0.3246, 0.1010, 0.0, 0.0, 0.0},
                    0, -9, 12.5, // TODO: find these! (this is the camera at the back)
                    180.0, 0.0, 0.0
            )
            // Add more cameras here...
    );

    // For fast lookup by name
    private static final Map<String, Camera> CAMERA_MAP = new HashMap<>();

    static {
        for (Camera cam : CAMERAS) {
            CAMERA_MAP.put(cam.name, cam);
        }
    }

    // Access a camera by its name
    public static Camera getByName(String name) {
        return CAMERA_MAP.get(name);
    }
}