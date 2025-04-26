package org.firstinspires.ftc.teamcode.framework.subsystems.digitaldevice;

/** A wrapper to make everything that attaches to digital ports look the same (otherwise they're all different in unique
 * and annoying ways) */
public interface DigitalDevice {

    class Parameters {
        /** Create a Parameters object and provide parameters that don't have default values.
         * @param deviceName The name of the device in the HardwareMap (robot configuration)
         */
        public Parameters(String deviceName) {
            this.deviceName = deviceName;
        }

        /** You already set this in the constructor and cannot set it again. */
        public final String deviceName;
        /** If this is true, the reported state of the device is inverted. */
        public boolean invert = false;
    }

    /** When a device is "active" depends on the implementation. */
    boolean isActive();
}
