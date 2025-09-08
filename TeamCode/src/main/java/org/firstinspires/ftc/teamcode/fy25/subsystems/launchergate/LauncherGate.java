package org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

/** Represents a launcher gate. */
public interface LauncherGate {

    /** You must set some of these if this subsystem is present. */
    class Parameters {
        /** Create a Parameters object and provide necessary parameters.
         * @param present Is this subsystem installed on this robot?
         */
        public Parameters(boolean present) {
            this.present = present;
        }

        /** You have already set this in the constructor and cannot set it again. */
        public final boolean present;

        /** The class of the actuator that will run the gate.
         * Example: for a Servo, this would be 'Servo.class'. <b>Has no default.</b> */
        public Class deviceClass;

        /** The actuator that will run the gate, already instantiated and configured. <b>Has no default.</b> */
        public HardwareDevice device;
    }

    /** Open the gate (start running actuator) */
    void open();
    /** Close the gate (stop running actuator) */
    void close();

    /** Is the gate open? */
    boolean isOpen();

    /** Set the power at which the actuator will run when the gate is open.
     * {@param power} The new power to set. */
    void setPower(double power);

    /** Get the power of the actuator. (whether it's set or current power may depend on actuator type) */
    int getPower();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
