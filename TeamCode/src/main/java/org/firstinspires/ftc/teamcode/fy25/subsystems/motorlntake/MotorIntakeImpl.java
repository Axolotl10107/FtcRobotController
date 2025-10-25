package org.firstinspires.ftc.teamcode.fy25.subsystems.motorlntake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;

public class MotorIntakeImpl implements MotorIntake{
    DcMotorEx motor;
    double intakeVel;
    final double tolerance = 20;
    State state = State.NONE;

    Telemetry telemetry;

    public MotorIntakeImpl(Parameters parameters) {
        motor = parameters.motor;
        intakeVel = parameters.IntakeTPS;
        telemetry = TelemetrySingleton.getInstance();
    }

    @Override
    public void spinIn() {
        motor.setVelocity(getIntakeVelocity());
    }

    @Override
    public void spinOut() {
        motor.setVelocity(-getIntakeVelocity());
    }

    @Override
    public void stop() {
        motor.setVelocity(0);
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
        telemetry.addData("Intake Motor", motor.getVelocity());
    }
}