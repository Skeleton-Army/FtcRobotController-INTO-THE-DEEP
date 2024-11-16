package org.firstinspires.ftc.teamcode.utils.actions;

import com.acmerobotics.roadrunner.Action;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.config.IntakeConfig;

public class Intake {
    private DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "extend"); //TODO: find out what the name is
    }

    public class MoveArm implements Action {
        private boolean initialized = false;
        private int targetPos;

        public MoveArm(int targetPos) {
            this.targetPos = targetPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                motor.setTargetPosition(targetPos);
                initialized = true;
            }

            return !motor.isBusy();
        }
    }

    public Action moveArm(int targetPos) {
        return new MoveArm(targetPos);
    }

}
