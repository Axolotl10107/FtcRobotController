package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.framework.adapters.DualMotor;

public class LauncherWheelImpl implements LauncherWheel {

    DcMotorEx motor;
    double launchVel;
    final double tolerance = 20;

    public LauncherWheelImpl(LauncherWheel.Parameters parameters) {
        motor = new DualMotor(parameters.motor1, parameters.motor2);
        launchVel = parameters.velocityRPM;
    }

    @Override
    public void spinUp() {
        motor.setVelocity(getLaunchVelocity(), AngleUnit.DEGREES);
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
//
//        if (Math.abs(getLaunchVelocity() - currentVel) >= tolerance) {
//            return State.READY;
//        } else if (currentVel < getLaunchVelocity()) {
//            if (motor.isMotorEnabled()) {
//                return State.STARTING;
//            } else {
//                if (currentVel >= tolerance) {
//                    return State.SLOWING;
//                } else {
//                    return State.STOPPED;
//                }
//            }
//        } else {
//            return State.ERROR;
//        }
        if (currentVel <= tolerance) {
            return State.STOPPED;
        } else {
            return State.RUNOUT;
        }
    }

    @Override
    public void setLaunchVelocity(double newVel) {
        launchVel = newVel;
    }

    @Override
    public double getLaunchVelocity() {
        return launchVel * 60;
    }

    @Override
    public void update() {

    }
}
