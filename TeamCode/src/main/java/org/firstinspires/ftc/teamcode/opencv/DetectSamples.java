package org.firstinspires.ftc.teamcode.opencv;

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

    public OpenCvCamera webcam;
    boolean viewportPaused; //Do we really need this?
    private final Telemetry telemetry;

    //yellow
    private final Scalar lowerBoundMask = new Scalar(0, 138, 0);
    private final Scalar upperBoundMask = new Scalar(255, 200, 100);

    private static final float epsilonConstant = 0.025f;
    private static final Size kernelSize = new Size(5, 5); //We try new one!

    public List<Sample> samples;


    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam){
        this.telemetry = telemetry;
        this.webcam = webcam;
    }

    public Mat processFrame(Mat input) {

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask(input), contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        samples = new ArrayList<>();

        for (MatOfPoint contour : contours) {
            //check if contour is a valid sample
            //if (contour.size().area() < 500 || contour.size().area() > 5000) //TODO: figure out what these constants should be
              //  continue;

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            double epsilon = epsilonConstant * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, contour2f, epsilon, true);
            Point[] vertices = contour2f.toArray();
            Sample tempname = new Sample(vertices);
            samples.add(tempname);
            telemetry.addData("X", tempname.getSampleX());
            telemetry.addData("Y", tempname.getSampleY());
            telemetry.addData("distance", tempname.getDistance());
        }

        telemetry.update();
        return input;
    }


    private Mat mask(Mat frame) {
        Mat masked = new Mat();
        Imgproc.cvtColor(frame, masked, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(masked, lowerBoundMask, upperBoundMask, masked);
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
