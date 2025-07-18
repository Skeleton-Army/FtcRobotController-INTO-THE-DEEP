package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

class SampleTest {
    static class Test {
        public Point input;
        public double x;
        public double y;

        public Test(Point input, double x, double y) {
            this.input = input;
            this.x = x;
            this.y = y;
        }
    }

    final Test[] values = new Test[] {
            new Test(new Point(320, 235), 0, 24),
            new Test(new Point(90, 200), 14.5, 28.5),
            //new Test(new Point(205, 333), 5, 14.5),
            new Test(new Point(66, 313), 11, 16.6),
            new Test(new Point(81, 184), 16.7, 31),
            new Test(new Point(190, 220), 7.5, 26),
            //new Test(new Point(440, 225), -6.5, 26),
            new Test(new Point(430, 283), -5, 19.75),
            new Test(new Point(373, 334), -2, 15.75),
    };

    final double epsilonX = 0.5;
    final double epsilonY = 0.5;

    @org.junit.jupiter.api.Test
    void testSampleDetection() {
        CameraConfig.offsetX = 0;
        CameraConfig.offsetY = 0;

        int index = 1;

        for (Test value : values ) {
            Sample sample = new Sample(value.input, value.input, new RotatedRect(), new Pose2d(0, 0, 0));

            double sampleX = sample.getSampleX();
            double sampleY = sample.getSampleY();

            assertTrue(Math.abs(sampleX - value.x) < epsilonX, "Value: " + index + " Expected: " + value.x + " Got: " + sampleX);
            assertTrue(Math.abs(sampleY - value.y) < epsilonY, "Value: " + index + " Expected: " + value.y + " Got: " + sampleY);

            index++;
        }
    }
}