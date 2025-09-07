package org.firstinspires.ftc.teamcode.framework.subsystems.claw;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice.BlankServo;

/** Represents a claw and its state (open or closed). */
public interface Claw {

    /** Represents the state of the claw. */
    enum State {
        /** Claw is open. */
        OPEN,
        /** Claw is closed. */
        CLOSED,
        /** Either no state has been set yet, or the claw should rest. */
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

        /** The servo that drives the claw, already instantiated and configured. Defaults to a {@link BlankServo}. */
        public Servo clawServo = new BlankServo();

        /** The servo position (between 0 and 1) that the claw is considered open. Defaults to the servo's position at
         * the time the constructor runs (so this subsystem should never move the servo if you don't set this). */
        public double openPosition = clawServo.getPosition();
        /** The servo position (between 0 and 1) that the claw is considered closed. Defaults to the servo's position at
         * the time the constructor runs (so this subsystem should never move the servo if you don't set this). */
        public double closePosition = clawServo.getPosition();
    }

    /** Set the desired state of the claw, and it will move to reach that state. */
    void setState(State state);
    /** Get the currently set state of the claw. */
    State getState();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
