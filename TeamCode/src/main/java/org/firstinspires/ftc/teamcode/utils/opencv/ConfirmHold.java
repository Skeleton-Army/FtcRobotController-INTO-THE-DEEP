package org.firstinspires.ftc.teamcode.utils.opencv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class ConfirmHold{
    Telemetry telemetry;
    public static  Rect roi = new Rect(100, 50, 20, 80);

    public ConfirmHold(Telemetry t){
        telemetry = t;
    }

    public void checkSampleHolding(Mat frame, SampleColor color){
        // Crop the image using the submat method
        Mat croppedImage = new Mat(frame, roi);
        Threshold t = new Threshold(color);
        Mat masked = new Mat();
        // Convert the frame to HSV color space
        Imgproc.cvtColor(croppedImage, masked, Imgproc.COLOR_RGB2YCrCb);
        Mat binary = Mat.zeros(croppedImage.size(), Imgproc.THRESH_BINARY);
        // Apply color filtering to isolate yellow objects
        Core.inRange(masked, t.lowerBound, t.upperBound, binary);
        telemetry.addData("Holding: ", Core.countNonZero(binary) > 0);
    }
}
