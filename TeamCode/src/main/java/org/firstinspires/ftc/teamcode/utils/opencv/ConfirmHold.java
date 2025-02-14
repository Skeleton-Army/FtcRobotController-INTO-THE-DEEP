package org.firstinspires.ftc.teamcode.utils.opencv;

import org.firstinspires.ftc.teamcode.utils.actionClasses.Intake;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

public class ConfirmHold{

   public static boolean success;

   public Intake intake;

    public ConfirmHold(Intake arm){
        success = false;
        intake = arm;
    }

    public static boolean checkSampleHolding(Mat frame, SampleColor color){
        // Define the rectangle (x, y, width, height) you want to crop
        int x = 100;   // X coordinate of the top-left corner of the cropping region
        int y = 50;    // Y coordinate of the top-left corner of the cropping region
        int width = 20;  // Width of the region to crop
        int height = 80; // Height of the region to crop

        // Create a Rect object with the cropping parameters
        Rect roi = new Rect(x, y, width, height);

        // Crop the image using the submat method
        Mat croppedImage = new Mat(frame, roi);
        Threshold threshold = new Threshold(color);


        return success;
    }


}
