package org.firstinspires.ftc.teamcode.fy25.subsystems.motorintake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;

public class MotorIntakeImpl implements MotorIntake{
    DcMotorEx motor;
    double intakeVel;
    final double tolerance = 20;
    State state = State.NONE;

    public MotorIntakeImpl(Parameters parameters) {
        motor = parameters.motor;
        intakeVel = parameters.IntakeTPS;
    }

    @Override
    public void spinIn() {
        motor.setPower(1);
    }

    @Override
    public void spinOut() {
        motor.setPower(-1);
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void setIntakeVelocity(double velocity) {
        intakeVel = velocity;
    }

    @Override
    public double getIntakeVelocity() {
        return intakeVel;
    }

    @Override
    public void update() {
    }
}