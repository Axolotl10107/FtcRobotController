package org.firstinspires.ftc.teamcode.fy23.fakestuff;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.DigitalDevice;

public class MockDigitalDevice implements DigitalDevice {

    private boolean active;

    public MockDigitalDevice(boolean initialState) {
        active = initialState;
    }

    @Override
    public boolean isActive() {
        return active;
    }

    public void setActive(boolean newState) {
        active = newState;
    }
}
