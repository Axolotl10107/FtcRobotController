package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LauncherWheelImpl implements LauncherWheel {

    DcMotorEx motor;
    double motorTPR;
    double launchVelBase;
    double launchVel;

    double launchVelTarget;
    final boolean isDynamic;
    final double tolerance;
    final double spinFactor = 1.25; // ratio between dynamic launchWheel and non-dynamic launchWheel at 0 distance
    final double distanceCoef = 1; // coefficient of distance in spin calculation

    // TODO: calibrate spinFactor and distanceCoef

    public LauncherWheelImpl(LauncherWheel.Parameters parameters) {
        motor = parameters.motor;
        motorTPR = parameters.motorTPR;
        launchVelBase = (parameters.velocityRPM * motorTPR) / 60;
        launchVel = launchVelBase;
        isDynamic = parameters.isDynamic;
        tolerance = parameters.velocityTolerance;
    }

    @Override
    public void spinUp() {
//        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchVelTarget = launchVel;
        motor.setMotorEnable();
    }

    @Override
    public void spinDown() {
//        motor.setDirection(DcMotorSimple.Direction.FORWARD);
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
//        if (currentVel <= tolerance) {
//            return State.STOPPED;
//        } else {
//            return State.RUNOUT;
//        }
    }

    @Override
    public void fixLaunchSpin(double distance) {
        if (isDynamic) {
            launchVel = launchVelBase / (spinFactor * ((distance / distanceCoef) + 1));
        } else {
            launchVel = launchVelBase;
        }
    }

    @Override
    public void denyEntry() {
        motor.setMotorEnable();
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setVelocity(-500);
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
    public double getLaunchVelTarget() {return launchVelTarget;}

    @Override
    public void update() {
        motor.setVelocity(launchVelTarget);
    }
}
