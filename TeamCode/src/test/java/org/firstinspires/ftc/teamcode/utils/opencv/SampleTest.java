package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.junit.jupiter.api.Assertions.*;

import org.opencv.core.Point;

import java.util.ArrayList;

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
            new Test(new Point(1,1), 0.2, 0.2),
            new Test(new Point(1,1), 0.2, 0.2),
            new Test(new Point(1,1), 0.2, 0.2),
            new Test(new Point(1,1), 0.2, 0.2)
    };

    double epsilon = 10;

    @org.junit.jupiter.api.Test
    void testSampleDetection() {
        for (Test value : values) {
            Sample sample = new Sample(new Point[]{ value.input });

            double sampleX = sample.getSampleX();
            double sampleY = sample.getSampleY();

            assertTrue(Math.abs(sampleX - value.x) < epsilon);
            assertTrue(Math.abs(sampleY - value.y) < epsilon);
        }

    }
}