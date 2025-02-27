package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.b;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.lowerBlue;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.lowerRed;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.lowerYellow;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.upperBlue;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.upperRed;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.upperYellow;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.config.SampleConfig;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DetectSamples extends OpenCvPipeline {
    private Threshold[] thresholds;
    //final int THRESHOLD = 200;
    public OpenCvCamera webcam;
    boolean viewportPaused; //Do we really need this? yes
    private final Telemetry telemetry;

    // Adjusted yellow HSV bounds
    private static final float epsilonConstant = 0.025f;
    private static final Size kernelSize = new Size(5, 5); //We try new one!
    private MecanumDrive drive;
    public List<Sample> samples = new ArrayList<>();

    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam, MecanumDrive drive, SampleColor color){
        this.telemetry = telemetry;
        this.webcam = webcam;
        this.drive = drive;
        thresholds = new Threshold[] {new Threshold(color)};
    }
    public DetectSamples(Telemetry telemetry, OpenCvCamera webcam, MecanumDrive drive, SampleColor color1, SampleColor color2){
        this.telemetry = telemetry;
        this.webcam = webcam;
        this.drive = drive;
        thresholds = new Threshold[] {new Threshold(color1), new Threshold(color2)};
    }

    private Point getLowestPoint(MatOfPoint contour) {
        Point[] points = contour.toArray();
        Point lowestPoint = points[0];

        for (Point point : points) {
            if (point.y > lowestPoint.y) {
                lowestPoint = point;
            }
        }

        return lowestPoint;
    }
    public Mat processFrame(Mat input) {
        List<Sample> samplesFrame = new ArrayList<>();
        List<MatOfPoint> contours = new ArrayList<>();

        //Mat rowsToBlack = input.rowRange(0, THRESHOLD);
        //rowsToBlack.setTo(new Scalar(0, 0, 0));
        Mat masked = mask(input);
        Imgproc.findContours(masked, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Core.bitwise_and(input, masked, input);
        masked.release();
        for (MatOfPoint contour : contours) {
            //check if contour is a valid sample
            //if (contour.size().area() < 500 || contour.size().area() > 5000) //TODO: figure out what these constants should be
            Point lowestPoint = getLowestPoint(contour);

            Rect boundingBox = Imgproc.boundingRect(contour);
            int center = boundingBox.x + boundingBox.width / 2;

            Sample tempName = new Sample(new Point(center, lowestPoint.y), drive.pose);
            Imgproc.drawMarker(input, new Point(center, lowestPoint.y), new Scalar(255,255,0));
            tempName.calculateOrientation(contour);
            tempName.calculateField();
            samplesFrame.add(tempName);

            //Imgproc.drawMarker(input, tempName.lowest, new Scalar(255,255,255));
        }
        samples = samplesFrame;
        telemetry.addData("x image: ", samplesFrame.get(0).lowest.x);
        telemetry.addData("y image: ", samplesFrame.get(0).lowest.y);

        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));
        return input;
    }

    private Mat mask(Mat frame) {
        Mat masked = new Mat();
        // Convert the frame to HSV color space
        Imgproc.cvtColor(frame, masked, Imgproc.COLOR_RGB2YCrCb);
        Mat binary = Mat.zeros(frame.size(), Imgproc.THRESH_BINARY);
        // Apply color filtering to isolate yellow objects
        for (Threshold t : thresholds) {
            Core.inRange(masked, t.lowerBound, t.upperBound, binary);
        }
        masked.release();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_CLOSE, kernel);
        return binary;
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
