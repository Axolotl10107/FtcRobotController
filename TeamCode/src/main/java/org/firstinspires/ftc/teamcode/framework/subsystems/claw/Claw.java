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
        /** Create a Parameters object and provide parameters that don't have default values.
         * @param present Is this subsystem installed on this robot?
         * @param openPosition The servo position (between 0 and 1) that the claw is considered open.
         * @param closePosition The servo position (between 0 and 1) that the claw is considered closed.
         */
        public Parameters(boolean present, double openPosition, double closePosition) {
            this.present = present;

            assert openPosition >= 0 && openPosition <= 1;
            this.openPosition = openPosition;

            assert closePosition >= 0 && closePosition <= 1;
            this.closePosition = closePosition;
        }

        /** You already set this in the constructor and cannot set it again. */
        public final boolean present;

        /** The servo that drives the claw, already instantiated and configured */
        public final Servo clawServo = new BlankServo();

        /** You already set this in the constructor and cannot set it again. */
        public final double openPosition;
        /** You already set this in the constructor and cannot set it again. */
        public final double closePosition;
    }

    /** Set the desired state of the claw, and it will move to reach that state. */
    void setState(State state);
    /** Get the currently set state of the claw. */
    State getState();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
