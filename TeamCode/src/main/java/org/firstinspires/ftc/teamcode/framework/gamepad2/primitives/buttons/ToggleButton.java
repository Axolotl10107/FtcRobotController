package org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.buttons;

import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.Button;

/** Toggles state on each press. */
public class ToggleButton implements Button {

    private final Button.BoolLambda button;
    private boolean state;
    private boolean latched = false;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * button field:
     * new ToggleButton( () -{@literal >} gamepad.x ); */
    public ToggleButton(Button.BoolLambda button) {
        this(button, false);
    }

    /** Same as the other constructor, but allows for setting the initial state. */
    public ToggleButton(Button.BoolLambda button, boolean initialState) {
        this.button = button;
        this.state = initialState;
    }

    /** Whether the button is active toggles each time it is pressed. */
    @Override
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
