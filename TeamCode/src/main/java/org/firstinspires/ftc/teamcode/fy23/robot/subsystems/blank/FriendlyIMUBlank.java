package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU;

/** A blank implementation of {@link FriendlyIMU} that does nothing. */
public class FriendlyIMUBlank implements FriendlyIMU {
    @Override
    public double pitch() {
        return 0;
    }

    @Override
    public double pitchVel() {
        return 0;
    }

    @Override
    public double roll() {
        return 0;
    }

    @Override
    public double rollVel() {
        return 0;
    }

    @Override
    public double yaw() {
        return 0;
    }

    @Override
    public double yawVel() {
        return 0;
    }

    @Override
    public void update() {

    }

}
