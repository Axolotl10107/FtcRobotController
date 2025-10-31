package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.framework.adapters.DualMotor;

public class LauncherWheelImpl implements LauncherWheel {

    DcMotorEx motor;
    double motorTPR;
    double launchVel;
    final double tolerance;

    public LauncherWheelImpl(LauncherWheel.Parameters parameters) {
        motor = parameters.motor;
        motorTPR = parameters.motorTPR;
        launchVel = (parameters.velocityRPM * motorTPR) / 60;
        tolerance = parameters.velocityTolerance;
    }

    @Override
    public void spinUp() {
        motor.setVelocity(launchVel);
        motor.setMotorEnable();
    }

    @Override
    public void spinDown() {
        // setting motor velocity to 0 𒁢
        motor.setVelocity(0);

        motor.setMotorDisable();
    }

    @Override
    public State getState() {
        double currentVel = motor.getVelocity();
//
        if (Math.abs(launchVel - currentVel) >= tolerance) {
            return State.READY;
        } else if (currentVel < launchVel) {
            if (motor.isMotorEnabled()) {
                return State.STARTING;
            } else {
                if (currentVel >= tolerance) {
                    return State.SLOWING;
                } else {
                    return State.STOPPED;
                }
            }
        } else {
            return State.ERROR;
        }
//        if (currentVel <= tolerance) {
//            return State.STOPPED;
//        } else {
//            return State.RUNOUT;
//        }
    }

    @Override
    public void setLaunchRPM(double newRPM) {
        // Divide by 60 to get rev.s per minute instead of rev.s per second
        launchVel = (newRPM * motorTPR) / 60;
    }

    @Override
    public double getLaunchRPM() {
        return launchVel / (motorTPR * 60);
    }

    @Override
    public void update() {

    }
}
