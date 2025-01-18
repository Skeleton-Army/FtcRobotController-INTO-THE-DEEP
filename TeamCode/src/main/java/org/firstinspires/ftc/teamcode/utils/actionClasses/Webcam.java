package org.firstinspires.ftc.teamcode.utils.actionClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.actions.BasketCycle;
import org.firstinspires.ftc.teamcode.utils.actions.PickupSample;
import org.firstinspires.ftc.teamcode.utils.actions.SpecimenCycle;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;

public class Webcam {

    Drive Actionsdrive;
    Intake intake;
    Outtake outtake;

    Apriltag apriltag;
    String alliance;
    public Webcam(Drive drive, Intake intake, Outtake outtake) {
        this.Actionsdrive = drive;
        this.intake = intake;
        this.outtake = outtake;
    }

    public Action basketCycle(String alliance) {
        return new BasketCycle(Actionsdrive, Actionsdrive.apriltag, outtake, alliance);
    }

    public Action specimenCycle() {
        return new SpecimenCycle();
    }

    public Action pickupSample(Vector2d targetSamplePos) {
        return new PickupSample(intake, Actionsdrive, targetSamplePos);
    }
}
