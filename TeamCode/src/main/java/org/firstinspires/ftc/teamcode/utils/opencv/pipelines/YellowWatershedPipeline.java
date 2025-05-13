package org.firstinspires.ftc.teamcode.utils.opencv.pipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class YellowWatershedPipeline extends OpenCvPipeline {

    Mat hsv = new Mat();
    Mat yellowMask = new Mat();
    Mat sureFg = new Mat();
    Mat sureBg = new Mat();
    Mat unknown = new Mat();
    Mat markers = new Mat();
    Mat distTrans = new Mat();
    Scalar lower = new Scalar(20, 100, 100);
    Scalar upper = new Scalar(30, 255, 255);

    Mat kernel;

    @Override
    public Mat processFrame(Mat input) {
        // 1. Convert to HSV and Threshold for Yellow
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar lower = new Scalar(20, 100, 100);
        Scalar upper = new Scalar(30, 255, 255);
        Core.inRange(hsv, lower, upper, yellowMask);

        // 2. Morphological Opening to Remove Noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);

        // 3. Dilate to get Sure Background
        Imgproc.dilate(yellowMask, sureBg, kernel, new Point(-1, -1), 3);

        // 4. Distance Transform and Threshold to get Sure Foreground
        Imgproc.distanceTransform(yellowMask, distTrans, Imgproc.DIST_L2, 5);
        Core.normalize(distTrans, distTrans, 0, 1.0, Core.NORM_MINMAX);
        Imgproc.threshold(distTrans, sureFg, 0.4, 1.0, Imgproc.THRESH_BINARY);
        sureFg.convertTo(sureFg, CvType.CV_8U);

        // 5. Subtract Sure FG from BG to get Unknown
        Core.subtract(sureBg, sureFg, unknown);

        // 6. Connected Components for Marker Labelling
        Imgproc.connectedComponents(sureFg, markers);
        Core.add(markers, Scalar.all(1), markers);
        for (int i = 0; i < unknown.rows(); i++) {
            for (int j = 0; j < unknown.cols(); j++) {
                if (unknown.get(i, j)[0] == 255) {
                    markers.put(i, j, 0);
                }
            }
        }

        // 7. Apply Watershed
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2BGR);  // Watershed requires 3-channel image
        Imgproc.watershed(input, markers);

        // 8. Draw results
        for (int i = 0; i < markers.rows(); i++) {
            for (int j = 0; j < markers.cols(); j++) {
                double[] val = markers.get(i, j);
                if (val[0] == -1) {
                    input.put(i, j, new double[]{255, 0, 255}); // Magenta line for boundaries
                }
            }
        }

        return input;
    }






}
