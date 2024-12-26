package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DetectSamples extends OpenCvPipeline {
    //final int THRESHOLD = 200;
    public OpenCvCamera webcam;
    boolean viewportPaused; //Do we really need this? yes
    private final Telemetry telemetry;

    // Adjusted yellow HSV bounds
    private Scalar lowerBound = new Scalar(18.4, 66.6, 111.9); // Lower bound for yellow
    private Scalar upperBound = new Scalar(32.6, 255, 255); // Upper bound for yellow


    private static final float epsilonConstant = 0.025f;
    private static final Size kernelSize = new Size(5, 5); //We try new one!

    public List<Sample> samples = new ArrayList<>();


    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam, SampleColor color){
        this.telemetry = telemetry;
        this.webcam = webcam;

        switch (color) {
            case RED:
                this.lowerBound = lowerRed;
                this.upperBound = upperRed;

            case BLUE:
                this.lowerBound = lowerYellow;
                this.upperBound = upperYellow;

            case YELLOW:
                this.lowerBound = lowerBlue;
                this.upperBound = upperBlue;
        }
    }

    public Mat processFrame(Mat input) {
        List<Sample> samplesFrame = new ArrayList<>();
        List<MatOfPoint> contours = new ArrayList<>();

        //Mat rowsToBlack = input.rowRange(0, THRESHOLD);
        //rowsToBlack.setTo(new Scalar(0, 0, 0));

        Imgproc.findContours(mask(input), contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            //check if contour is a valid sample
            //if (contour.size().area() < 500 || contour.size().area() > 5000) //TODO: figure out what these constants should be
              //  continue;
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            double epsilon = epsilonConstant * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, contour2f, epsilon, true);
            Point[] vertices = contour2f.toArray();
            Sample tempName = new Sample(vertices);
            samplesFrame.add(tempName);
            telemetry.addData("X", tempName.getSampleX());
            telemetry.addData("Y", tempName.getSampleY());
            telemetry.addData("distance", tempName.getDistance());

            Imgproc.drawMarker(input, tempName.reference, new Scalar(255,255,255));
            contour2f.release();
        }
        samples = samplesFrame;
        telemetry.update();
        return input;
    }


    private Mat mask(Mat frame) {
        Mat masked = new Mat();

        // Convert the frame to HSV color space
        Imgproc.cvtColor(frame, masked, Imgproc.COLOR_BGR2HSV);

        // Apply color filtering to isolate yellow objects
        Core.inRange(masked, lowerBound, upperBound, masked);

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
