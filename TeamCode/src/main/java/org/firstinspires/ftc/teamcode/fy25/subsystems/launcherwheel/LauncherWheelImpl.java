package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LauncherWheelImpl implements LauncherWheel {

    final DcMotorEx motor;
    final double motorTPR;
    final double launchVelBase;
    final double denyVel;
    final double tolerance;
//    final double spinFactor; // ratio between dynamic launchWheel and non-dynamic launchWheel at 0 distance
//    final double distanceCoef; // coefficient of distance in spin calculation

    double launchVel;
    double launchVelTarget;

    public LauncherWheelImpl(LauncherWheel.Parameters parameters) {
        motor = parameters.motor;
        motorTPR = parameters.motorTPR;
        launchVelBase = (parameters.velocityRPM * motorTPR) / 60;
        launchVel = launchVelBase;
        denyVel = parameters.denyVel;
        tolerance = parameters.velocityTolerance;
//        this.spinFactor = parameters.spinFactor;
//        this.distanceCoef = parameters.distanceCoef;
    }

    @Override
    public void spinUp() {
        launchVelTarget = launchVel;
        motor.setMotorEnable();
    }

    @Override
    public void spinDown() {
        launchVelTarget = 0;
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
    }

//    @Override
//    public void fixLaunchSpin(double distance) {
//        launchVel = launchVelBase / (spinFactor * ((distance / distanceCoef) + 1));
//    }
//
//    @Override
//    public void revertLaunchSpin() {
//        launchVel = launchVelBase;
//    }

    @Override
    public void allowEntry() {
        motor.setMotorEnable();
        motor.setTargetPosition(motor.getCurrentPosition() + 10);
    }

    @Override
    public void setLaunchRPM(double newRPM) {
        // Divide by 60 to get rev.s per minute instead of rev.s per second
        launchVel = (newRPM * motorTPR) / 60;
    }

    @Override
    public double getCurrentRPM() {
        return motor.getVelocity() / (motorTPR * 60);
    }

    @Override
    public double getLaunchVelTargetRPM() {
        return launchVelTarget / (motorTPR * 60);
    }

    @Override
    public void update() {
        motor.setVelocity(launchVelTarget);
    }
}
