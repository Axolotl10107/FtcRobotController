package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice.BlankMotor;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheel;
import org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice.BlankMotor;

public interface LauncherWheelSimple {
    /** Represents the state of the launcher. */
    enum State {
        ON,
        OFF
    }

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

        public DcMotor motor = new BlankMotor();
    }

    /** Apply power to the launch wheel. */
    void spinUp();
    /** Remove power from the launch wheel. */
    void spinDown();


    /** Called by robot.update(). You do not need to call this method. */
    void update();
}
