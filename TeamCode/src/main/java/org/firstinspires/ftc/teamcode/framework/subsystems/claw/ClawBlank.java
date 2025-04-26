package org.firstinspires.ftc.teamcode.framework.subsystems.claw;

/** A blank implementation of {@link Claw} that does nothing.
 * Never returns null (returns blank objects instead where applicable), so hopefully no null pointers. */
public class ClawBlank implements Claw {

    @Override
    public void setState(State state) {

    }

    @Override
    public State getState() {
        return State.NONE;
    }

    @Override
    public void update() {

    }

}
