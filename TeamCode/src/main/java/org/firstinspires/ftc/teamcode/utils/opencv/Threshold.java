package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.*;

import org.opencv.core.Scalar;
import java.util.ArrayList;
import java.util.List;

public class Threshold {
    public SampleColor color;
    public List<Scalar> lowerBounds;
    public List<Scalar> upperBounds;

    public Threshold(SampleColor color) {
        this.color = color;
        this.lowerBounds = new ArrayList<>();
        this.upperBounds = new ArrayList<>();

        switch (color) {
            case YELLOW:
                lowerBounds.add(lowerYellow);
                upperBounds.add(upperYellow);
                break;

            case RED:
                lowerBounds.add(lowerRed1);
                upperBounds.add(upperRed1);
                lowerBounds.add(lowerRed2);
                upperBounds.add(upperRed2);
                break;

            case BLUE:
                lowerBounds.add(lowerBlue);
                upperBounds.add(upperBlue);
                break;
        }
    }
}
