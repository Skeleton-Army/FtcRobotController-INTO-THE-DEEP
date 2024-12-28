package org.firstinspires.ftc.teamcode.opModes.tests.teleop;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.actionClasses.Drive;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.teamcode.utils.general.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.teleop.TeleopOpMode;

/*
    the following test will ensure that robot can utilize the apriltag to move across the field
    namely , do spline to the apriltag he detects
 */

public class GoToApriltag extends TeleopOpMode {

    MecanumDrive drive;

    Apriltag apriltag;

    Drive driveActions;

    boolean updatePos = false;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        apriltag = new Apriltag(hardwareMap, drive);
        apriltag.enableApriltag();

        driveActions = new Drive(drive, apriltag);
    }

    @Override
    public void init_loop() {
        if (Apriltag.getCurrentDetections() != null) {
            // setting the position
            apriltag.updateRobotPos(Apriltag.getCurrentDetections().get(0));
            updatePos = true;
        }

        telemetry.addData("Robot Position is updated? ",updatePos);
        telemetry.addLine();
        telemetry.addData("Robot position: ", drive.pose);

        telemetry.update();
    }

    @Override
    public void start() {

        runAction(driveActions.moveApriltag(new Pose2d(0,0,0)));
    }

    @Override
    public void loop() {

    }
}
