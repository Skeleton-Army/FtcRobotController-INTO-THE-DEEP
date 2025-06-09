package org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint;

import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.AvoidSubParametersConfig.height;
import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.AvoidSubParametersConfig.fieldSize;
import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.AvoidSubParametersConfig.heightOffset;
import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.AvoidSubParametersConfig.obstacles;
import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.AvoidSubParametersConfig.width;
import static org.firstinspires.ftc.teamcode.opModes.tests.autonomous.BezierToPoint.AvoidSubParametersConfig.widthOffset;

import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class BezierToPoint2 {

    public Pose beginPose;
    public Pose endPose;

    public Point midPoint;

    public Telemetry telemetry;
    public static boolean useTelemetry;
    public static List<Point> path;
    public BezierToPoint2(Pose beginPose, Pose endPose, boolean useTelemetry ,Telemetry telemetry) {
        this.beginPose = beginPose;
        this.endPose = endPose;

        midPoint = adjustMidpointToAvoid(
                new Point(beginPose.getX(), beginPose.getY()),
                new Point(endPose.getX(), endPose.getY()),
                beginPose.getHeading(),
                endPose.getHeading(),
                obstacles,
                "widest", // <-- try "fastest", "shortest", or "widest"
                2,
                telemetry,
                "right"
        );


        if (midPoint == null) {
            midPoint = new Point((beginPose.getX() + endPose.getX()) / 2, (beginPose.getY() + endPose.getY()) / 2 + 30);
        }

        BezierToPoint2.useTelemetry = useTelemetry;
        path = bezierCurve(new Point[]{new Point(beginPose.getX(), beginPose.getY()), midPoint, new Point(endPose.getX(), endPose.getY())}, 1000);

        if (useTelemetry && telemetry != null) {
            telemetry.addData("Final Midpoint", "x: %.2f, y: %.2f", midPoint.x, midPoint.y);
            telemetry.addData("Path Points", path.size());
            telemetry.update();
        }
    }

    public static List<Point> getPath() {
        return path;
    }

    public static class Point {
        public double x, y;
        public Point(double x, double y) { this.x = x; this.y = y; }

        public double distanceTo(Point other) {
            return Math.hypot(this.x - other.x, this.y - other.y);
        }
    }

    public static double[][] rotatePolygon(double[][] points, double angleRad) {
        double cos = Math.cos(angleRad);
        double sin = Math.sin(angleRad);
        double[][] rotated = new double[points.length][2];

        for (int i = 0; i < points.length; i++) {
            double x = points[i][0];
            double y = points[i][1];
            rotated[i][0] = x * cos - y * sin;
            rotated[i][1] = x * sin + y * cos;
        }

        return rotated;
    }

    public static boolean pointInPolygon(double[] point, double[][] polygon) {
        int crossings = 0;
        for (int i = 0; i < polygon.length; i++) {
            double[] a = polygon[i];
            double[] b = polygon[(i + 1) % polygon.length];

            if (((a[1] > point[1]) != (b[1] > point[1])) &&
                    (point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0])) {
                crossings++;
            }
        }
        return (crossings % 2 == 1);
    }

    public static double minDistanceBetweenPolygons(double[][] poly1, double[][] poly2) {
        double minDist = Double.MAX_VALUE;
        for (double[] p1 : poly1) {
            for (double[] p2 : poly2) {
                double dx = p1[0] - p2[0];
                double dy = p1[1] - p2[1];
                double dist = Math.hypot(dx, dy);
                if (dist < minDist) minDist = dist;
            }
        }
        return minDist;
    }

    public static boolean isOverlapping(Point center, double angle, List<double[][]> obstacles) {
        double cx = center.x, cy = center.y;
        double dx = (double)(width + widthOffset) / 2, dy = (double)(height + heightOffset) / 2;
        double[][] corners = new double[][] {
                {-dx, -dy}, {dx, -dy}, {dx, dy}, {-dx, dy}
        };

        double[][] rotated = rotatePolygon(corners, angle);
        for (int i = 0; i < rotated.length; i++) {
            rotated[i][0] += cx;
            rotated[i][1] += cy;
        }

        for (double[][] obs : obstacles) {
            for (double[] corner : rotated) {
                if (pointInPolygon(corner, obs)) return true;
            }

            if (minDistanceBetweenPolygons(rotated, obs) < 5.0) return true;
        }

        for (double[] corner : rotated) {
            if (corner[0] < 0 || corner[0] > fieldSize || corner[1] < 0 || corner[1] > fieldSize) return true;
        }

        return false;
    }

    public static Point bezierPoint(Point[] controlPoints, double t) {
        int n = controlPoints.length - 1;
        Point result = new Point(0, 0);
        for (int i = 0; i <= n; i++) {
            double binomial = binomialCoeff(n, i);
            double coeff = binomial * Math.pow(t, i) * Math.pow(1 - t, n - i);
            result.x += coeff * controlPoints[i].x;
            result.y += coeff * controlPoints[i].y;
        }
        return result;
    }

    public static double binomialCoeff(int n, int k) {
        double res = 1;
        for (int i = 0; i < k; ++i) {
            res *= (n - i);
            res /= (i + 1);
        }
        return res;
    }

    public static List<Point> bezierCurve(Point[] controlPoints, int numPoints) {
        List<Point> curve = new ArrayList<>();
        for (int i = 0; i < numPoints; i++) {
            double t = i / (double)(numPoints - 1);
            curve.add(bezierPoint(controlPoints, t));
        }
        return curve;
    }

    public static Point adjustMidpointToAvoid(
            Point start, Point end,
            double startAngle, double endAngle,
            List<double[][]> obstacles,
            String preference,
            int topN,
            Telemetry telemetry,
            String preferredSide
    ) {
        Point direction = new Point(end.x - start.x, end.y - start.y);
        double mag = Math.hypot(direction.x, direction.y);
        direction.x /= mag;
        direction.y /= mag;

        Point perpendicular = new Point(-direction.y, direction.x);
        Point mid = new Point((start.x + end.x) / 2, (start.y + end.y) / 2);

        // Finer and wider offset test range
        List<Double> offsetMagnitudes = new ArrayList<>();
        for (int i = -200; i <= 200; i += 5) {
            offsetMagnitudes.add((double) i);
        }

        double bestShortest = Double.POSITIVE_INFINITY;
        double bestFastest = Double.POSITIVE_INFINITY;
        double bestClearance = -1;

        Point bestMidShortest = null;
        Point bestMidFastest = null;
        Point bestMidWidest = null;

        List<Point[]> validCandidates = new ArrayList<>();

        for (double offset : offsetMagnitudes) {
            if ("left".equals(preferredSide) && offset > 0) continue;
            if ("right".equals(preferredSide) && offset < 0) continue;
            Point testMid = new Point(mid.x + perpendicular.x * offset, mid.y + perpendicular.y * offset);
            Point[] ctrlPts = new Point[]{start, testMid, end};
            List<Point> candidatePath = bezierCurve(ctrlPts, 500);

            boolean collision = false;
            double minClearance = Double.POSITIVE_INFINITY;

            for (int i = 0; i < candidatePath.size(); i++) {
                Point pt = candidatePath.get(i);
                double angle = startAngle + (endAngle - startAngle) * i / candidatePath.size();
                if (isOverlapping(pt, angle, obstacles)) {
                    collision = true;
                    break;
                }
            }

            if (!collision) {
                double length = 0;
                double curvaturePenalty = 0;

                for (int i = 1; i < candidatePath.size(); i++) {
                    double dx1 = candidatePath.get(i).x - candidatePath.get(i - 1).x;
                    double dy1 = candidatePath.get(i).y - candidatePath.get(i - 1).y;
                    length += Math.hypot(dx1, dy1);
                    if (i > 1) {
                        double dx0 = candidatePath.get(i - 1).x - candidatePath.get(i - 2).x;
                        double dy0 = candidatePath.get(i - 1).y - candidatePath.get(i - 2).y;
                        double angle1 = Math.atan2(dy0, dx0);
                        double angle2 = Math.atan2(dy1, dx1);
                        double delta = Math.abs(angle2 - angle1);
                        if (delta > Math.PI) delta = 2 * Math.PI - delta;
                        curvaturePenalty += delta;
                    }
                }

                // Calculate clearance (minimum distance to any obstacle)
                for (double[][] obs : obstacles) {
                    double[][] robotCorners = rotatePolygon(new double[][]{
                            {-width / 2.0, -height / 2.0},
                            {width / 2.0, -height / 2.0},
                            {width / 2.0, height / 2.0},
                            {-width / 2.0, height / 2.0}
                    }, 0); // angle 0 for midpoint

                    for (int j = 0; j < robotCorners.length; j++) {
                        robotCorners[j][0] += testMid.x;
                        robotCorners[j][1] += testMid.y;
                    }

                    double clearance = minDistanceBetweenPolygons(robotCorners, obs);
                    if (clearance < minClearance) minClearance = clearance;
                }

                validCandidates.add(ctrlPts);

                double fastCost = length + 100 * curvaturePenalty;

                if (length < bestShortest) {
                    bestShortest = length;
                    bestMidShortest = testMid;
                }

                if (fastCost < bestFastest) {
                    bestFastest = fastCost;
                    bestMidFastest = testMid;
                }

                if (minClearance > bestClearance) {
                    bestClearance = minClearance;
                    bestMidWidest = testMid;
                }
            }
        }

        // Debug output
        if (useTelemetry && telemetry != null) {
            telemetry.addData("Checked Candidates", offsetMagnitudes.size());
            telemetry.addData("Valid Midpoints", validCandidates.size());

            if (validCandidates.size() > 0) {
                for (int i = 0; i < Math.min(topN, validCandidates.size()); i++) {
                    Point midpt = validCandidates.get(i)[1];
                    telemetry.addData("Midpoint #" + (i + 1), "x: %.2f, y: %.2f", midpt.x, midpt.y);
                }
            } else {
                telemetry.addLine("No valid path candidates found!");
            }

            telemetry.update();
        }

        if (validCandidates.isEmpty()) return null;

        // Final selection logic
        if ("fastest".equals(preference) && bestMidFastest != null) return bestMidFastest;
        if ("shortest".equals(preference) && bestMidShortest != null) return bestMidShortest;
        if ("widest".equals(preference) && bestMidWidest != null) return bestMidWidest;

        // Fallback
        if (bestMidFastest != null) return bestMidFastest;
        if (bestMidShortest != null) return bestMidShortest;
        if (bestMidWidest != null) return bestMidWidest;

        return null;
    }

}
