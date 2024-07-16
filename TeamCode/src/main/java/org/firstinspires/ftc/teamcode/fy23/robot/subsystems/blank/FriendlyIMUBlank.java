package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU;

/** A blank implementation of {@link FriendlyIMU} that does nothing.
 * Never returns null (returns blank objects instead where applicable), so hopefully no null pointers. */
public class FriendlyIMUBlank implements FriendlyIMU {
    @Override
    public double pitch() {
        return 0;
    }

    @Override
    public double pitch(AngleUnit angleUnit) {
        return 0;
    }

    @Override
    public double pitchVel() {
        return 0;
    }

    @Override
    public double pitchVel(AngleUnit angleUnit) {
        return 0;
    }

    @Override
    public double roll() {
        return 0;
    }

    @Override
    public double roll(AngleUnit angleUnit) {
        return 0;
    }

    @Override
    public double rollVel() {
        return 0;
    }

    @Override
    public double rollVel(AngleUnit angleUnit) {
        return 0;
    }

    @Override
    public double yaw() {
        return 0;
    }

    @Override
    public double yaw(AngleUnit angleUnit) {
        return 0;
    }

    @Override
    public double yawVel() {
        return 0;
    }

    @Override
    public double yawVel(AngleUnit angleUnit) {
        return 0;
    }

    @Override
    public void update() {

    }

}
