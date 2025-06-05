package org.firstinspires.ftc.teamcode.utils.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.hardware.Servo;

public class FollowPath implements Action {
    private final Follower follower;
    private final Path path;
    private final PathChain pathChain;
    private final boolean holdEnd;

    private boolean isFollowing = false;

    public FollowPath(Follower follower, Path path) {
        this.follower = follower;
        this.path = path;
        this.pathChain = null;
        this.holdEnd = true;
    }

    public FollowPath(Follower follower, Path path, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.pathChain = null;
        this.holdEnd = holdEnd;
    }

    public FollowPath(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.path = null;
        this.pathChain = pathChain;
        this.holdEnd = true;
    }

    public FollowPath(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.follower = follower;
        this.path = null;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!isFollowing) {
            if (path != null) follower.followPath(path, holdEnd);
            if (pathChain != null) follower.followPath(pathChain, holdEnd);

            isFollowing = true;
        }

        return follower.isBusy();
    }
}
