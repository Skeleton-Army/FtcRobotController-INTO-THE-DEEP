package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.autoTeleop.AprilTagSamplesPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class MoveApriltag implements Action {
    AprilTagSamplesPipeline AprilTagSamplesPipeline;
    Pose2d targetPose;

    MecanumDrive drive;

    public MoveApriltag(Pose2d targetPose, MecanumDrive drive, AprilTagSamplesPipeline AprilTagSamplesPipeline) {
        this.targetPose = targetPose;
        this.drive = drive;
        this.AprilTagSamplesPipeline = AprilTagSamplesPipeline;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        AprilTagDetection detections = AprilTagSamplesPipeline.getApriltagDetection(); // gets the first detection that it sees

        if (detections != null) {

            // if the target position is the origin, the target position will be the Apriltag's position on the field
            if (targetPose.equals(new Pose2d(0,0,0)))
                this.targetPose = new Pose2d(detections.rawPose.x - 6, detections.rawPose.y - 6,0);

        }
        // do a spline to the target apriltag, in this case the first one that was detected
        Actions.runBlocking(
                drive.actionBuilder(AprilTagSamplesPipeline.getRobotPosByAprilTag())
                        .splineToLinearHeading(targetPose, 0)
                        .build()
        );
        return false;
    }
}
