package org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.buttons;

import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.Button;

/** Toggles state on each press. */
public class ToggleButton implements Button {

    private Button.BoolLambda button;
    private boolean state = false;
    private boolean latched = false;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * button field:
     * new ToggleButton( () -{@literal >} gamepad.x ); */
    public ToggleButton(Button.BoolLambda button) {
        this.button = button;
    }

    /** Same as the other constructor, but allows for setting the initial state. */
    public ToggleButton(Button.BoolLambda button, boolean initialState) {
        this.button = button;
        this.state = initialState;
    }

    @Override
    /** Returns whether the button should be considered active. */
    public boolean isActive() {
        if (button.get() && !latched) {
            latched = true;
            state = !state;
        } else if (!button.get() && latched) {
            latched = false;
        }
        return state;
    }
}
