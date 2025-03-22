package org.firstinspires.ftc.teamcode.utils.opencv;

import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.lowerBlue;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.lowerRed;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.lowerYellow;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.upperBlue;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.upperRed;
import static org.firstinspires.ftc.teamcode.utils.config.SampleConfig.upperYellow;

import org.opencv.core.Scalar;

public class Threshold {
    public Scalar lowerBound;
    public Scalar upperBound;
    public Threshold(SampleColor color) {
        switch (color) {
            case YELLOW:
                this.lowerBound = lowerYellow;
                this.upperBound = upperYellow;
                break;

            case RED:
                this.lowerBound = lowerRed;
                this.upperBound = upperRed;
                break;

            case BLUE:
                this.lowerBound = lowerBlue;
                this.upperBound = upperBlue;
                break;
        }
    }
}
