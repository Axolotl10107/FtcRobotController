package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LauncherWheelImpl implements LauncherWheel {

    DcMotorEx motor;
    double launchVel;
    final double tolerance = 20;

    public LauncherWheelImpl(LauncherWheel.Parameters parameters) {
        motor = parameters.motor;
        launchVel = parameters.velocityTPS;
        spinDown();
    }

    @Override
    public void spinUp() {
        motor.setVelocity(getLaunchVelocity());
        motor.setMotorEnable();
    }

    @Override
    public void spinDown() {
        motor.setMotorDisable();

        // setting motor velocity to 0 𒁢
        motor.setVelocity(0);
    }

    @Override
    public State getState() {
        double currentVel = motor.getVelocity();

        if (Math.abs(getLaunchVelocity() - currentVel) >= tolerance) {
            return State.READY;
        } else if (currentVel < getLaunchVelocity()) {
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

    @Override
    public void setLaunchVelocity(double newVel) {
        launchVel = newVel;
    }

    @Override
    public double getLaunchVelocity() {
        return launchVel;
    }

    @Override
    public void update() {

    }
}
