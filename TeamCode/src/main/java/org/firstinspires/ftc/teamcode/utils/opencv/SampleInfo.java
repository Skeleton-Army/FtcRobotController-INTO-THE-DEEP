package org.firstinspires.ftc.teamcode.utils.opencv;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

public class SampleInfo {
    private final Sample sample;
    private final Pose2d currPose;
    private double extendTarget;
    private double distance;
    private double turnAngle;

    public SampleInfo(Sample sample, Pose2d currPose) {
        this.sample = sample;
        this.currPose = currPose;
        calculate();
    }

    private void calculate() {
        Vector2d sampleVec = sample.getSamplePosition().position;
        Vector2d relative = sampleVec.minus(currPose.position);
        this.distance = relative.norm();
        this.turnAngle = Math.atan2(relative.y, relative.x) + Math.acos(IntakeConfig.offsetFromCenterX / distance) - Math.PI / 2;
        this.extendTarget = Math.sqrt(Math.pow(distance, 2) - Math.pow(IntakeConfig.offsetFromCenterX, 2));
    }

    public double getIntakeRotation() {
        double orientation = Math.toDegrees(turnAngle) - sample.orientation;
        double normalizedOrientation = (90 - Math.abs(orientation)) * Math.signum(orientation);
        return normalizedOrientation / 90;
    }

    public Sample getSample() {
        return sample;
    }

    public double getTurnAngle() {
        return turnAngle;
    }

    public boolean isReachable() {
        return ((int) (extendTarget * IntakeConfig.tickOverInch) <= IntakeConfig.extendPosition);
    }

    public double getDistance() {
        return distance;
    }

    public int getExtendTarget() {
        return (int)extendTarget;
    }
}
