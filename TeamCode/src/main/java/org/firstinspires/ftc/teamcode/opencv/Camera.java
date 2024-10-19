package org.firstinspires.ftc.teamcode.opencv;

public final class Camera {

    //public static final int h_fov = 78;
    //public static final float v_fov = 41.4f;
    //public static final float inchLength = 1.5f;
    //public static final int focalLength = 272;
    //public static final int imageWidth = 320;
    //public static final int imageHeight = 240;

    public static final float z = 9.4f; //TODO: figure out what this is
    public static final int inchXfocal = 408;
    public static final float hOVERwidth = 0.24375f;
    public static final float vOVERheight = 0.1725f;
    public static final int halfImageWidth = 160;
    public static final int halfImageHeight = 120;
}

//double x = Camera.inchXfocal / calculatePixels(vertices);
//double y = Math.tan(Math.toRadians(angle)) * x;
/*
public double calculatePixels(Point[] vertices) {
        telemetry.addData("Vertices", vertices.length);
        if (vertices.length == 6) {
        double sum = 0;
        for (int i = 0; i < vertices.length; i++) {
        Point start = vertices[i];
        Point end = vertices[(i + 1) % vertices.length];
        double length = Math.sqrt(Math.pow(start.x - end.x, 2) + Math.pow(start.y - end.y, 2));
        if (Math.abs(start.x - end.x) < 0.2 * length) {
        sum += length;
        }
        }
        return sum / 2;
        }

        else if (vertices.length == 4) {
        for (int i = 0; i < vertices.length; i++) {
        Point start = vertices[i];
        Point end = vertices[(i + 1) % vertices.length];
        double length = Math.sqrt(Math.pow(start.x - end.x, 2) + Math.pow(start.y - end.y, 2));
        if (Math.abs(start.x - end.x) < 0.2 * length) {
        double wanted_length = length / (1 + Math.tan(Math.toRadians((vertices[0].y - Camera.halfImageHeight) * Camera.vOVERheight))); //I saw this wasn't up to date with what we did on 15.10, recovered from what I remembered
        double horizontal_length = Math.sqrt(Math.pow(vertices[(i + 2) % vertices.length].x - vertices[(i + 3) % vertices.length].x, 2) + Math.pow(vertices[(i + 2) % vertices.length].y - vertices[(i + 3) % vertices.length].y, 2));
        if (Math.abs(2.33 * wanted_length - horizontal_length) < horizontal_length * 0.2)
        return wanted_length;
        return horizontal_length;
        }
        }
        }

        else if (vertices.length == 5){
        //I think the same thing will work for the 3 vertices case, but I won't put it here yet until we check so not to cause more bugs

        //What if it recognizes 5 vertices but when there should be 6?
        //Raviv: did we encounter that case?

        double minX = 0, minY = 0, maxX = 0, maxY = 0;
        for (Point vertex : vertices) {
        minX = Math.min(minX, vertex.x);
        minY = Math.min(minY, vertex.y);
        maxX = Math.min(maxX, vertex.x);
        maxY = Math.min(maxY, vertex.y);
        }

        //we said the order doesn't matter, right? anyway please verify this
        Point[] corrected = {new Point(minX, maxY), new Point(maxX, maxY), new Point(maxX, minY), new Point(minX, minY)};
        return calculatePixels(corrected);
        }

        return -1;
        }
        */

/*
public double getY(Point[] vertices) {
        double lowest = vertices[0].y;
        for (Point vertex : vertices) {
            if (vertex.y > lowest) {
                lowest = vertex.y;
            }
        }
        return Camera.z / Math.tan(Math.toRadians((lowest - Camera.halfImageHeight) * Camera.vOVERheight));
    }
 */