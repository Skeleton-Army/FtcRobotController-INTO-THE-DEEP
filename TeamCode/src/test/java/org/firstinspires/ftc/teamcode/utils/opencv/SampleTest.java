package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.Point;

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
            new Test(new Point(324, 238), 0, 23.5),
            new Test(new Point(252, 238), 4, 23.5),
            new Test(new Point(146, 238), 10, 23.5),
            new Test(new Point(93, 238), 13, 23.5),
    };

    final double epsilon = 1;

    @org.junit.jupiter.api.Test
    void testSampleDetection() {
        CameraConfig.offsetX = 0;
        CameraConfig.offsetY = 0;

        int index = 1;

        for (Test value : values ) {
            Sample sample = new Sample(value.input, new Pose2d(0, 0, 0));

            double sampleX = sample.getSampleX();
            double sampleY = sample.getSampleY();

            assertTrue(Math.abs(sampleX - value.x) < epsilon, "Value: " + index + " Expected: " + value.x + " Got: " + sampleX);
            assertTrue(Math.abs(sampleY - value.y) < epsilon, "Value: " + index + " Expected: " + value.y + " Got: " + sampleY);

            index++;
        }
    }
}