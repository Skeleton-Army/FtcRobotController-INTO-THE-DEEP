package org.firstinspires.ftc.teamcode.utils.actions;

import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.d;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.f;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.i;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.p;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.power;
import static org.firstinspires.ftc.teamcode.utils.config.SpecimenArmConfig.target;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SpecimenArmPIDF  implements Action {
    private final DcMotorEx motor;

    public SpecimenArmPIDF(DcMotorEx motor) {
        this.motor = motor;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        while  (target - 10 > motor.getCurrentPosition() && motor.getCurrentPosition() > target + 10) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,new PIDFCoefficients(p, i, d, f));
            motor.setTargetPosition(target);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Use this instead for RUN_USING_ENCODER
            //motor.setVelocityPIDFCoefficients(p, i, d, f);
            //        motor.setPositionPIDFCoefficients(tP);
            //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motor.setPower(power);
        }

        return false;
    }
}
