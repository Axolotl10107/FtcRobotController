package org.firstinspires.ftc.teamcode.fy23.fakestuff;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU;

public class MockFriendlyIMU implements FriendlyIMU {

    private double pitch = 0;
    private double pitchVel = 0;

    private double roll = 0;
    private double rollVel = 0;

    private double yaw = 0;
    private double yawVel = 0;

    @Override
    public double pitch() {
        return pitch;
    }

    @Override
    public double pitchVel() {
        return pitchVel;
    }

    @Override
    public double roll() {
        return roll;
    }

    @Override
    public double rollVel() {
        return rollVel;
    }

    @Override
    public double yaw() {
        return yaw;
    }

    @Override
    public double yawVel() {
        return yawVel;
    }

    @Override
    public void update() {

    }

    // Methods for use in your test
    public void setPitch(double pitch) {
        this.pitch = pitch;
    }
    public void setPitchVel(double pitchVel) { this.pitchVel = pitchVel; }

    public void setRoll(double roll) {
        this.roll = roll;
    }
    public void setRollVel(double rollVel) { this.rollVel = rollVel; }

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }
    public void setYawVel(double yawVel) { this.yawVel = yawVel; }
}
