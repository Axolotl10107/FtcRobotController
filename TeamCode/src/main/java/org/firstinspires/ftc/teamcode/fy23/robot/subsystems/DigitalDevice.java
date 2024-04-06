package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

/** A wrapper to make everything that attaches to digital ports look the same (otherwise they're all different in unique
 * and annoying ways) */
public interface DigitalDevice {

    class Parameters {
        /** The name of the device in the HardwareMap (robot configuration) */
        public String deviceName;
        /** If this is true, the reported state of the device is inverted. */
        public boolean invert;
    }

    /** Given the implementation and the parameters you set, should the state of this device be considered active (true)
     * right now? */
    boolean isActive();
}
