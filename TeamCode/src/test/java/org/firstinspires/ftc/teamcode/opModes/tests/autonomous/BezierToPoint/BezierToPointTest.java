package org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint;

import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.AvoidSubParametersConfig.obstacles;
import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.BezierToPoint.Point;
import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.BezierToPoint.isOverlapping;
import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.BezierToPoint.testMid;
import static org.junit.jupiter.api.Assertions.fail;

import com.pedropathing.localization.Pose;

import java.util.List;

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
                if (isOverlapping(new BezierToPoint.Point(path.get(i).x, path.get(i).y) , angle, obstacles)) {
                    collision = true;
                    break;
                }
            }
            return collision;
        }
    }

    final BezierToPointTest.Test[] values = new BezierToPointTest.Test[] {
          //new BezierToPointTest.Test(new Pose(48, 24, Math.toRadians(90)), new Pose(48, 120, Math.toRadians(0))),
         // new BezierToPointTest.Test(new Pose(24, 24, Math.toRadians(90)), new Pose(48, 120, Math.toRadians(0))),
           //new BezierToPointTest.Test(new Pose(48, 120, Math.toRadians(0)), new Pose(48, 24, Math.toRadians(90))),
          // new BezierToPointTest.Test(new Pose(72, 24, Math.toRadians(90)), new Pose(24, 120, Math.toRadians(0))),
          // new BezierToPointTest.Test(new Pose(96, 24, Math.toRadians(90)), new Pose(24, 120, Math.toRadians(0))),
           // new BezierToPointTest.Test(new Pose(120, 24, Math.toRadians(90)), new Pose(24, 120, Math.toRadians(0))),

           new BezierToPointTest.Test(new Pose(24, 24, Math.toRadians(90)), new Pose(72, 110, Math.toRadians(0))),

            //new BezierToPointTest.Test(new Pose(48, 24, Math.toRadians(0)), new Pose(48, 120, Math.toRadians(0))),
            //new BezierToPointTest.Test(new Pose(120, 24, Math.toRadians(90)), new Pose(95, 120, Math.toRadians(0))),
            //new BezierToPointTest.Test(new Pose(120, 24, Math.toRadians(0)), new Pose(24, 120, Math.toRadians(0))),
            //new BezierToPointTest.Test(new Pose(72, 24, Math.toRadians(0)), new Pose(72, 120, Math.toRadians(0))),
            //new BezierToPointTest.Test(new Pose(72, 24, Math.toRadians(0)), new Pose(24, 72, Math.toRadians(0))),
            //new BezierToPointTest.Test(new Pose(75, 37, Math.toRadians(90)), new Pose(14, 48, Math.toRadians(0))),
    };


    @org.junit.jupiter.api.Test
    void bezierPath() {
        int index = 1;


        for (BezierToPointTest.Test value : values ) {

            BezierToPoint bezier = new BezierToPoint(value.beginPose, value.endPose, false, null);

            Point p = new Point((value.beginPose.getX() + value.endPose.getX()) / 2, (value.beginPose.getY() + value.endPose.getY()) / 2);

            if ((bezier.midPoint != null) && (bezier.midPoint.x != p.x) || (bezier.midPoint.y != p.y)) {
                System.out.println("-------------");
                System.out.println(bezier.midPoint.x);
                System.out.println(bezier.midPoint.y);
                System.out.println("-------------");
            }

            else {
                System.out.println("-------------");
                System.out.println("failed to find a mid point!");
                System.out.println("-------------");

                System.out.println(bezier.midPoint.x);
                System.out.println(bezier.midPoint.y);
                System.out.println(p.x);
                System.out.println(p.y);

                System.out.println(testMid.x);
                System.out.println(testMid.y);



                fail();
            }
            index++;
        }
    }
}
