package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.buttons;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Button;

/** Active when held down and not active when up. Can be inverted. */
public class MomentaryButton implements Button {

    private BoolLambda button;
    private boolean invert = false;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * button field:
     * new MomentaryButton( () -> gamepad.x ); */
    public MomentaryButton(BoolLambda button) {
        this.button = button;
    }

    /** Same as the other constructor, but allows for inverting the active state. If invert is set
     * to true, then isActive() returns true when the button is up and false when the button is down. */
    public MomentaryButton(BoolLambda button, boolean invert) {
        this.button = button;
        this.invert = invert;
    }

    @Override
    /** Returns whether the button should be considered active. */
    public boolean isActive() {
        return invert ^ button.get();
        /*
        XOR ( ^ ) - "exclusive or" - either A or B, but not both
        invert | button.get() | result
        0 | 0 | 0
        0 | 1 | 1
        1 | 0 | 1
        1 | 1 | 0
        */
    }
}
