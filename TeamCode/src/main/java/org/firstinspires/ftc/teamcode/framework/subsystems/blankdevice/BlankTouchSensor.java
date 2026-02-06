package org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class BlankTouchSensor extends BlankHardwareDevice implements TouchSensor {
    @Override
    public double getValue() {
        return 0;
    }

    @Override
    public boolean isPressed() {
        return false;
    }
}
