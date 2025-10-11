package org.firstinspires.ftc.teamcode.fy25.subsystems.MotorIntake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.LauncherGateMotorImpl;

public class MotorIntakeImpl implements MotorIntake{
    DcMotorEx motor;
    double intakeVel;
    final double tolerance = 20;

    public MotorIntakeImpl(Parameters parameters) {
        motor = parameters.motor;
        intakeVel = parameters.IntakeTPS;
    }

    @Override
    public void spinIn() {
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setVelocity(getIntakeVelocity());
    }

    @Override
    public void spinOut() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setVelocity(getIntakeVelocity());
    }

    @Override
    public void stop() {
        motor.setVelocity(0);
    }

    @Override
    public State getState() {
        if (Math.abs(motor.getVelocity() - intakeVel) <= tolerance) {
            if (motor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                return State.RUNIN;
            } else {
                return State.RUNOUT;
            }
        } else {
            return State.NONE;
        }
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
