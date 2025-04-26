package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.hardwaredevice.BlankCRServo;

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

    class Parameters {
        /** Create a Parameters object and provide parameters that don't have default values.
         * @param present Is this subsystem installed on this robot?
         */
        public Parameters(boolean present) {
            this.present = present;
        }

        /** You already set this in the constructor and cannot set it again. */
        public final boolean present;

        /** The servo that drives the claw, already instantiated and configured */
        public CRServo intakeServo = new BlankCRServo();

        public double servoPower = 1;
    }

    /** Set the desired state of the claw, and it will move to reach that state. */
    void setState(State state);
    /** Get the currently set state of the claw. */
    State getState();

    void setPower(double power);

    double getPower();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
