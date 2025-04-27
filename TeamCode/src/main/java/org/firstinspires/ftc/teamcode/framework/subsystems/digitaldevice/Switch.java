package org.firstinspires.ftc.teamcode.framework.subsystems.digitaldevice;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** An implementation of DigitalDevice for simple switches or other simple uses of {@link DigitalChannel}. */
public class Switch implements DigitalDevice {

    private final DigitalChannel device;
    private final boolean invert;

    public Switch(HardwareMap hardwareMap, DigitalDevice.Parameters parameters) {
        device = hardwareMap.get(DigitalChannel.class, parameters.deviceName);
        invert = parameters.invert;
    }

    @Override
    public boolean isActive() {
        if (invert) {
            return !device.getState();
        } else {
            return device.getState();
        }
    }

}
