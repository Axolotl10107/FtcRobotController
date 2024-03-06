package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.buttons;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Button;

/** Only active on the initial press. Can trigger when button goes up or down. */
public class TriggerButton implements Button {

    private Button.BoolLambda button;
    private boolean invert = false;
    private boolean latched = false;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * button field:
     * new TriggerButton( () -{@literal >} gamepad.x ); */
    public TriggerButton(Button.BoolLambda button) {
        this.button = button;
    }

    /** Same as the other constructor, but allows for inverting the active state. If invert is set
     * to true, then isActive() returns true when the button is up and false when the button is down. */
    public TriggerButton(Button.BoolLambda button, boolean invert) {
        this.button = button;
        this.invert = invert;
    }

    @Override
    /** Returns whether the button should be considered active. */
    public boolean isActive() {
        boolean active = invert ^ button.get();
        if (active && !latched) {
            latched = true;
            return active;
        } else if (!active && latched) {
            latched = false;
            return active;
        } else {
            return false;
        }
    }

}
