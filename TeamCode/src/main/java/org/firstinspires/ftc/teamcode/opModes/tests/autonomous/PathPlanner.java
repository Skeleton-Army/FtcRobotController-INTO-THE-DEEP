/*
package org.firstinspires.ftc.teamcode.opModes.tests.autonomous;

import com.acmerobotics.dashboard.canvas.Polygon;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;

import java.util.ArrayList;
import java.util.List;

public class PathPlanner {

    private static final int FIELD_SIZE = 144;
    private static final GeometryFactory geometryFactory = new GeometryFactory();

    public static Vector2d[] bezierCurve(Vector2d[] controlPoints, int numPoints) {
        int n = controlPoints.length - 1;
        Vector2d[] curve = new Vector2d[numPoints];
        for (int i = 0; i < numPoints; i++) {
            double t = i / (double) (numPoints - 1);
            double x = 0;
            double y = 0;
            for (int j = 0; j <= n; j++) {
                double bern = bernstein(j, n, t);
                x += bern * controlPoints[j].getX();
                y += bern * controlPoints[j].getY();
            }
            curve[i] = new Vector2d(x, y);
        }
        return curve;
    }

    private static double bernstein(int i, int n, double t) {
        return comb(n, i) * Math.pow(t, i) * Math.pow(1 - t, n - i);
    }

    private static int comb(int n, int k) {
        if (k == 0 || k == n) return 1;
        int res = 1;
        for (int i = 1; i <= k; i++) {
            res *= (n - (k - i));
            res /= i;
        }
        return res;
    }

    private static boolean isOverlapping(Vector2d center, double width, double height, double angleDeg, List<Polygon> obstacles) {
        double cx = center.x;
        double cy = center.y;
        double dx = width / 2;
        double dy = height / 2;

        double[][] corners = {
                {-dx, -dy}, {dx, -dy}, {dx, dy}, {-dx, dy}
        };

        double angleRad = Math.toRadians(angleDeg);
        double cos = Math.cos(angleRad);
        double sin = Math.sin(angleRad);

        Coordinate[] coords = new Coordinate[5];
        for (int i = 0; i < 4; i++) {
            double x = corners[i][0];
            double y = corners[i][1];
            double rx = cos * x - sin * y + cx;
            double ry = sin * x + cos * y + cy;
            coords[i] = new Coordinate(rx, ry);
        }
        coords[4] = coords[0]; // close the polygon

        Polygon robotPoly = geometryFactory.createPolygon(coords);
        Polygon fieldBox = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(0, 0),
                new Coordinate(FIELD_SIZE, 0),
                new Coordinate(FIELD_SIZE, FIELD_SIZE),
                new Coordinate(0, FIELD_SIZE),
                new Coordinate(0, 0)
        });

        for (Polygon obs : obstacles) {
            if (robotPoly.intersects(obs)) return true;
        }
        return !fieldBox.contains(robotPoly);
    }

    public static PathChain generatePathChain(Vector2d start, Vector2d end, double rectW, double rectH,
                                              double startAngle, double endAngle, List<Polygon> obstacles,
                                              String preference, int topN) {

        Vector2d mid = new Vector2d((start.getX() + end.getX()) / 2, (start.getY() + end.getY()) / 2);
        double[] offsetRange = java.util.stream.IntStream.range(-25, 25)
                .mapToDouble(i -> i * 3.2).toArray(); // from -80 to 80 approx

        List<Candidate> validCandidates = new ArrayList<>();
        double bestLength = Double.MAX_VALUE;
        double bestFastCost = Double.MAX_VALUE;
        Vector2d bestLengthMid = null;
        Vector2d bestFastMid = null;

        for (double dx : offsetRange) {
            for (double dy : offsetRange) {
                Vector2d testMid = new Vector2d(mid.getX() + dx, mid.getY() + dy);
                Vector2d[] ctrlPts = {start, testMid, end};
                Vector2d[] path = bezierCurve(ctrlPts, 50);

                boolean hasCollision = false;
                for (int i = 0; i < path.length; i++) {
                    double angle = startAngle + (endAngle - startAngle) * i / path.length;
                    if (isOverlapping(path[i], rectW, rectH, angle, obstacles)) {
                        hasCollision = true;
                        break;
                    }
                }

                if (!hasCollision) {
                    double totalLength = 0;
                    double curvature = 0;
                    for (int i = 1; i < path.length; i++) {
                        double dx1 = path[i].getX() - path[i - 1].getX();
                        double dy1 = path[i].getY() - path[i - 1].getY();
                        totalLength += Math.hypot(dx1, dy1);
                        if (i > 1) {
                            double dx0 = path[i - 1].getX() - path[i - 2].getX();
                            double dy0 = path[i - 1].getY() - path[i - 2].getY();
                            double angle1 = Math.atan2(dy0, dx0);
                            double angle2 = Math.atan2(dy1, dx1);
                            double dAngle = Math.abs(angle2 - angle1);
                            dAngle = (dAngle + Math.PI) % (2 * Math.PI) - Math.PI;
                            curvature += Math.abs(dAngle);
                        }
                    }
                    double fastCost = totalLength + 100 * curvature;
                    validCandidates.add(new Candidate(totalLength, fastCost, testMid));

                    if (totalLength < bestLength) {
                        bestLength = totalLength;
                        bestLengthMid = testMid;
                    }
                    if (fastCost < bestFastCost) {
                        bestFastCost = fastCost;
                        bestFastMid = testMid;
                    }
                }
            }
        }

        Vector2d selectedMid;
        if ("shortest".equals(preference) && bestLengthMid != null) {
            selectedMid = bestLengthMid;
        } else if (bestFastMid != null) {
            selectedMid = bestFastMid;
        } else if (bestLengthMid != null) {
            selectedMid = bestLengthMid;
        } else {
            selectedMid = mid;
        }

        Vector2d[] finalCurve = bezierCurve(new Vector2d[]{start, selectedMid, end}, 50);
        return convertToPathChain(finalCurve, startAngle, endAngle);
    }

    private static PathChain convertToPathChain(Vector2d[] bezierCurve, double startAngle, double endAngle) {
        List<SplineSegment> segments = new ArrayList<>();
        Vector2d startVec = bezierCurve[0];
        Pose pose = new Pose(startVec, Math.toRadians(startAngle));

        for (int i = 1; i < bezierCurve.length; i++) {
            Vector2d next = bezierCurve[i];
            segments.add(new SplineSegment(pose, next));

            if (i < bezierCurve.length - 1) {
                double dx = bezierCurve[i + 1].getX() - bezierCurve[i].getX();
                double dy = bezierCurve[i + 1].getY() - bezierCurve[i].getY();
                double angle = Math.atan2(dy, dx);
                pose = new Pose(next, angle);
            } else {
                pose = new Pose(next, Math.toRadians(endAngle));
            }
        }

        return new PathChain(segments);
    }

    private static class Candidate {
        double length, fastCost;
        Vector2d midpoint;

        Candidate(double l, double f, Vector2d m) {
            length = l;
            fastCost = f;
            midpoint = m;
        }
    }
}

*/
