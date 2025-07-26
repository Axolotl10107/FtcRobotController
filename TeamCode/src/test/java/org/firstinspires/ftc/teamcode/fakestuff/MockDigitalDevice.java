package org.firstinspires.ftc.teamcode.fakestuff;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;
import org.firstinspires.ftc.teamcode.framework.subsystems.digitaldevice.DigitalDevice;

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
