package org.firstinspires.ftc.teamcode.opencv;

public final class Camera {

    //public static final int h_fov = 78;
    //public static final float v_fov = 41.4f;
    //public static final float inchLength = 1.5f;
    //public static final int focalLength = 272;
    //public static final int imageWidth = 320;
    //public static final int imageHeight = 240;

    public static final double z = -1; //TODO: figure out what this is
    public static final int inchXfocal = 408;
    public static final float hOVERwidth = 0.24375f;
    public static final float vOVERheight = 0.1725f;
    public static final int halfImageWidth = 160;
    public static final int halfImageHeight = 120;
}


//Code we are not currently using but are planning on using them soon:
/*
    // calculates the orientation of the sample based on the bottom/top of the sample i don't know if it works yet
    public double calculateOrientation(Point point1, Point point2) {
        double verticalAngle = calculateVerticalAngle(point1.y);
        return Math.toDegrees(Math.atan((point1.y - point2.y) / ((point1.x - point2.x) * Math.tan(Math.toRadians(verticalAngle)))));
    }
*/