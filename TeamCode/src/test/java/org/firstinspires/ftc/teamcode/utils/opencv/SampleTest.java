package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.utils.config.CameraConfig;
import org.opencv.core.Point;

class SampleTest {
    class Test {
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
            new Test(new Point(320,319), 0, 47.5),
            new Test(new Point(354,348), -2.5, 39.5),
            new Test(new Point(412,292), -11.5, 59.5),
            new Test(new Point(237,309), 9, 52),
            new Test(new Point(258,359), 5, 37.5),
    };

    double epsilon = 1.6;

    @org.junit.jupiter.api.Test
    void testSampleDetection() {
        CameraConfig.offsetHorizontal = 0;
        CameraConfig.offsetVertical = 0;

        int index = 1;

        for (Test value : values ) {
            Sample sample = new Sample(new Point[]{ value.input });

            double sampleX = sample.getSampleX();
            double sampleY = sample.getSampleY();

            assertTrue(Math.abs(sampleX - value.x) < epsilon, "Value: " + index + " Expected: " + value.x + " Got: " + sampleX);
            assertTrue(Math.abs(sampleY - value.y) < epsilon, "Value: " + index + " Expected: " + value.y + " Got: " + sampleY);

            index++;
        }
    }
}