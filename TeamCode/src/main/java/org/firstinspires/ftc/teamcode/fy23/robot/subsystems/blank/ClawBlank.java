package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;

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
