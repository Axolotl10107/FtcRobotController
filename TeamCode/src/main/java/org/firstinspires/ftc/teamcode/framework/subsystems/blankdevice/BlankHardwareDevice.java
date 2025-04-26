package org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public class BlankHardwareDevice implements HardwareDevice {
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "BlankHardwareDevice";
    }

    @Override
    public String getConnectionInfo() {
        return "Not Applicable";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
