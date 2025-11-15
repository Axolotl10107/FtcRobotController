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
        /** Launch wheel is slowing */
        SLOWING,
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

        /** The <b>velocity</b> at which the launch wheel will run, in <b>revolutions per minute</b>. Defaults to 0. */
        public double velocityRPM = 0;

        /** The velocity to run at when denyEntry() is called. */
        public double denyVel = 0;

        /** The tolerance around the launch velocity, within which the wheel can be considered READY, in <b>ticks per second</b>. Defaults to 20. */
        public double velocityTolerance = 20;

        /** The <b>ticks per revolution</b> of your motor (537.7 for a 312 RPM goBILDA 5203, for example). Defaults to 0. */
        public double motorTPR = 0;

//        /** For two LauncherWheels. Ratio between dynamic LauncherWheel and non-dynamic LauncherWheel. */
//        public double spinFactor = 1.25;
//
//        /** Coefficient of distance in spin calculation for fixLaunchSpin(). */
//        public double distanceCoef = 1.0;
    }

    /** Apply power to the launch wheel. */
    void spinUp();
    /** Remove power from the launch wheel. */
    void spinDown();

    /** Get the current state of the launcher.
     * This will be STARTING immediately after calling readyLauncher(),
     * and will become READY once the launch wheel is up to speed. */
    State getState();

    /** Set the velocity at which the launch wheel will run when it is READY.
     * {@param velocity} The new velocity to set, in <b>ticks per second</b>. */
    void setLaunchRPM(double velocity);

//    /** Adjust the launch velocity to aim for a certain distance from the launcher. */
//    void fixLaunchSpin(double distance);
//
//    /** Return to just the launch RPM you set instead of what was calculated by fixLaunchSpin() */
//    void revertLaunchSpin();

    /** Use the wheel to prevent an object from entering the launcher.
     * Not needed if you have a normal LauncherGate setup. */
    void denyEntry();

    /** Get the current velocity of the launch wheel in <b>RPM</b>. */
    double getCurrentRPM();

    /** Get the launch velocity that the wheel is aiming for in <b>RPM</b>. */
    double getLaunchVelTargetRPM();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
