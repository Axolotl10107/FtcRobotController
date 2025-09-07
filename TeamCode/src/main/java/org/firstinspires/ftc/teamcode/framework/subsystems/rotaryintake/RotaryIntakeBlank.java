package org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake;

/** A blank implementation of {@link RotaryIntake} that does nothing.
 * Never returns null (returns blank objects instead where applicable), so hopefully no null pointers. */
public class RotaryIntakeBlank implements RotaryIntake {

    @Override
    public void setState(State state) {

    }

    @Override
    public State getState() {
        return State.NONE;
    }

    @Override
    public void setPower(double power) {

    }

    @Override
    public double getPower() {
        return 0;
    }

    @Override
    public void update() {

    }
}
