package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.utils.actions.BasketCycle;
import org.firstinspires.ftc.teamcode.utils.actions.PickupSample;
import org.firstinspires.ftc.teamcode.utils.actions.SpecimenCycle;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.opencv.Sample;

public class Webcam {

    Drive Actionsdrive;
    Intake intake;
    Outtake outtake;

    Apriltag apriltag;
    String alliance;
    public Webcam(Drive drive, Intake intake, Outtake outtake, String alliance) {
        this.Actionsdrive = drive;
        this.intake = intake;
        this.outtake = outtake;
        this.alliance = alliance;
    }

    public Action basketCycle() {
        return new BasketCycle(Actionsdrive, outtake, alliance);
    }

    public Action specimenCycle() {
        return new SpecimenCycle();
    }

    public Action pickupSample(Sample targetSample) {
        return new PickupSample(intake, Actionsdrive, targetSample);
    }
}
