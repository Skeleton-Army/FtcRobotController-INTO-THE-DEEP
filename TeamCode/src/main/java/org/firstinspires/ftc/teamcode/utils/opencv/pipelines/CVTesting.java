package org.firstinspires.ftc.teamcode.utils.opencv.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CVTesting extends OpenCvPipeline {
    //final int THRESHOLD = 200;
    public OpenCvCamera webcam;
    boolean viewportPaused; //Do we really need this? yes

    // Adjusted yellow HSV bounds
    private Scalar lowerBound = new Scalar(0, 0, 0); // Lower bound for yellow
    private Scalar upperBound = new Scalar(255, 255, 255); // Upper bound for yellow

    private static final Size kernelSize = new Size(5, 5); //We try new one!

    public CVTesting() {

    }

    public Mat processFrame(Mat input) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat m = mask(input);
        Imgproc.findContours(mask(input), contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(m, contours,-1, new Scalar(255, 255, 255), -1);
        return m;
    }


    private Mat mask(Mat frame) {
        Mat masked = new Mat();

        // Apply color filtering to isolate yellow objects
        Core.inRange(frame, lowerBound, upperBound, masked);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_CLOSE, kernel);

        return masked;
    }


    //please explain this function
    @Override
    public void onViewportTapped()
    {
        viewportPaused = !viewportPaused;

        if (viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }
}

