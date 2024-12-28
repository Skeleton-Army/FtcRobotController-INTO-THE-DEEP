package org.firstinspires.ftc.teamcode.utils.actionClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.Apriltag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//TODO: do stuff here sometime!!
public class Drive {

    MecanumDrive drive;

    HardwareMap hardwareMap;
    public Drive(MecanumDrive drive) {
        this.drive = drive;
    }

    public class MoveApriltag implements Action {
        Apriltag apriltag = new Apriltag(hardwareMap, drive);
        Pose2d targetPose;

        public MoveApriltag(Pose2d targetPose) {
            this.targetPose = targetPose;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            apriltag.enableApriltag();

            AprilTagDetection detections = apriltag.getCurrentDetections().get(0);

            // if the target position is the origin, the target position will be the Apriltag's position on the field
            if (targetPose.equals(new Pose2d(0,0,0)))
                this.targetPose = new Pose2d(detections.rawPose.x - 6, detections.rawPose.y - 6,0);


            // do a spline to the target apriltag, in this case the first one that was detected
            Actions.runBlocking(
                    drive.actionBuilder(apriltag.getRobotPos(detections))
                            .splineToLinearHeading(targetPose, 0)
                            .build()
            );

            return false;
        }
    }

    public Action moveApriltag(Pose2d targetPose) {
        return new MoveApriltag(targetPose);

    }

}
