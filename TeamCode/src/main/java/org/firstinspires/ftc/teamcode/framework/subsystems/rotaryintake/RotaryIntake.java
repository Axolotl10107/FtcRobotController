package org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice.BlankCRServo;

/** Represents a rotary intake and its state (Running? In which direction?). */
public interface RotaryIntake {

    /** Represents the state of the intake. */
    enum State {
        /** Intake is intaking */
        RUNIN,
        /** Intake is outtaking */
        RUNOUT,
        /** Intake is stopped **/
        STOPPED,
        /** No state has been set yet*/
        NONE
    }

    /** You must set some of these if this subsystem is present. */
    class Parameters {
        /** Create a Parameters object and provide necessary parameters.
         * @param present Is this subsystem installed on this robot?
         */
        public Parameters(boolean present) {
            this.present = present;
        }

        /** You already set this in the constructor and cannot set it again. */
        public final boolean present;

        /** The servo that drives the claw, already instantiated and configured. Defaults to a {@link BlankCRServo}. */
        public CRServo intakeServo = new BlankCRServo();

        /** The power level at which the servo will run. Defaults to 1. */
        public double servoPower = 1;
    }

    /** Set the desired state of the intake, and it will move to reach that state.
     * {@param state} The new state to set. */
    void setState(State state);
    /** Get the currently set state of the intake.
     * @return {@link State} of the intake. */
    State getState();

    /** Set the power level at which the servo will run.
     * {@param power} The new power level to set. */
    void setPower(double power);

    /** Get the power level at which the servo runs when the intake is in an active state.
     * @return a double between 0 and 1. */
    double getPower();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
