package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice.BlankMotor;

/** Represents a launcher wheel and its state. */
public interface LauncherWheel {

    /** Represents the state of the launcher. */
    enum State {
        /** Launch wheel is coming up to speed */
        STARTING,
        /** Launch wheel is at speed. Ready for launch! */
        READY,
        /** Launch wheel is stopped. */
        STOPPED,
        /** Launch wheel is not behaving as it should */
        ERROR
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

        /** The {@link DcMotorEx} that runs the launch wheel, already instantiated and configured. Defaults to a {@link BlankMotor}. */
        public DcMotorEx motor = new BlankMotor();

        /** The <b>velocity</b> at which the launch wheel will run, in <b>ticks per second</b>. Defaults to 0. */
        public int velocityTPS = 0;
    }

    /** Apply power to the launch wheel */
    void spinUp();
    /** Remove power from the launch wheel */
    void spinDown();

    /** Get the current state of the launcher.
     * This will be STARTING immediately after calling readyLauncher(),
     * and will become READY once the launch wheel is up to speed. */
    State getState();

    /** Set the velocity at which the launch wheel will run when it is READY.
     * {@param velocity} The new velocity to set, in <b>ticks per second</b>. */
    void setVelocity(int velocity);

    /** Get the current velocity of the launch wheel. */
    int getVelocity();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
