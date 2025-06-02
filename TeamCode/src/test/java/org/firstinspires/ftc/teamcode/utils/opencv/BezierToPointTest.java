package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.AvoidSubParametersConfig.obstacles;
import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.BezierToPoint2.isOverlapping;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.BezierToPoint2;

public class BezierToPointTest {

    static class Test {

        public static Pose beginPose;
        public static Pose endPose;

        public Test(Pose beginPose, Pose endPose) {
            this.beginPose = beginPose;
            this.endPose = endPose;
        }

        public static boolean checkCollision(List<Point> path) {
            boolean collision = false;
            for (int i = 0; i < path.size(); i++) {
                double angle = beginPose.getHeading() + (endPose.getHeading() - beginPose.getHeading()) * i / path.size();
                if (isOverlapping(new BezierToPoint2.Point(path.get(i).getX(), path.get(i).getY()) , angle, obstacles)) {
                    collision = true;
                    break;
                }
            }
            return collision;
        }
    }

    final BezierToPointTest.Test[] values = new BezierToPointTest.Test[] {
            new BezierToPointTest.Test(new Pose(48, 24, Math.toRadians(0)), new Pose(48, 120, Math.toRadians(0))),
            //new BezierToPointTest.Test(new Pose(48, 120, Math.toRadians(0)), new Pose(48, 24, Math.toRadians(90))),
            //new BezierToPointTest.Test(new Pose(72.4, 22.6, Math.toRadians(90)), new Pose(23.8, 120, Math.toRadians(0))),
            //new BezierToPointTest.Test(new Pose(24, 24, Math.toRadians(90)), new Pose(72, 110, Math.toRadians(0))),
    };


    @org.junit.jupiter.api.Test
    void testSampleDetection() {
        int index = 1;

        for (BezierToPointTest.Test value : values ) {
            /*Sample sample = new Sample(value.input, value.input, new RotatedRect(), new Pose(0, 0, 0));

            double sampleX = sample.getSampleX();
            double sampleY = sample.getSampleY();

            assertTrue(Math.abs(sampleX - value.x) < epsilonX, "Value: " + index + " Expected: " + value.x + " Got: " + sampleX);
            assertTrue(Math.abs(sampleY - value.y) < epsilonY, "Value: " + index + " Expected: " + value.y + " Got: " + sampleY);*/

            BezierToPoint2 bezier = new BezierToPoint2(Test.beginPose, Test.endPose, false, null);

            System.out.println("-------------");
            System.out.println(bezier.midPoint.x);
            System.out.println(bezier.midPoint.y);
            System.out.println("-------------");
            //assertTrue(bezier.isColliding(Test.beginPose.getHeading(), Test.endPose.getHeading()));


            index++;
        }
    }
}
