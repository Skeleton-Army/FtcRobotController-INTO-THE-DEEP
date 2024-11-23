package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.TrajectoryActionStub;
import com.noahbres.meepmeep.roadrunner.entity.TurnActionStub;

import java.util.Arrays;

public class CustomDriveShim  {
    private final VelConstraint velConstraint;
    private final ProfileAccelConstraint accelConstraint;
    private final Constraints constraints;
    public Pose2d poseEstimate;

    public CustomDriveShim(DriveTrainType driveTrainType, Constraints constraints, Pose2d poseEstimate) {
        this.poseEstimate = poseEstimate;
        this.accelConstraint = new ProfileAccelConstraint(-constraints.getMaxAccel(), constraints.getMaxAccel());
        this.constraints = constraints;

        switch (driveTrainType) {
            case MECANUM:
                this.velConstraint = new MinVelConstraint(Arrays.asList(
                        new AngularVelConstraint(constraints.getMaxAngVel()),
                        new MecanumKinematics(constraints.getTrackWidth()).new WheelVelConstraint(constraints.getMaxVel())
                ));
                break;
            case TANK:
                this.velConstraint = new MinVelConstraint(Arrays.asList(
                        new AngularVelConstraint(constraints.getMaxAngVel()),
                        new TankKinematics(constraints.getTrackWidth()).new WheelVelConstraint(constraints.getMaxVel())
                ));
                break;
            default:
                throw new IllegalArgumentException("Unsupported drive train type");
        }
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return new TrajectoryActionBuilder(
                TurnActionStub::new,
                TrajectoryActionStub::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                startPose, 0.0,
                new TurnConstraints(
                        constraints.getMaxAngVel(),
                        -constraints.getMaxAngAccel(),
                        constraints.getMaxAngAccel()
                ),
                velConstraint,
                accelConstraint
        );
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d startPose, boolean mirrored) {
        // Mirror the trajectory if mirrored is true.
        PoseMap poseMap = mirrored ? pose -> new Pose2dDual<>(
                pose.position.x.unaryMinus(),
                pose.position.y.unaryMinus(),
                pose.heading.times(Rotation2d.exp(Math.toRadians(180)))
        ) : new IdentityPoseMap();

        return new TrajectoryActionBuilder(
                TurnActionStub::new,
                TrajectoryActionStub::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                startPose, 0.0,
                new TurnConstraints(
                        constraints.getMaxAngVel(),
                        -constraints.getMaxAngAccel(),
                        constraints.getMaxAngAccel()
                ),
                velConstraint,
                accelConstraint,
                poseMap
        );
    }
}
