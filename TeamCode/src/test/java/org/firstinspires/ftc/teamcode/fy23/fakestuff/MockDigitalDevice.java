package org.firstinspires.ftc.teamcode.fy23.fakestuff;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Button;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.DigitalDevice;

/** Can also be used as a mock {@link Button}. */
public class MockDigitalDevice implements DigitalDevice, Button {

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
