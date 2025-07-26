package org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;

/** Only active on new presses. Button must be released and pressed again to
 * become active again.
 * Can trigger when button goes up or down. */
public class TriggerButton implements Button {

    private final Button.BoolLambda button;
    private final boolean invert;
    private boolean latched = false;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * button field:
     * new TriggerButton( () -{@literal >} gamepad.x ); */
    public TriggerButton(Button.BoolLambda button) {
        this(button, false);
    }

    /** Same as the other constructor, but allows for inverting the active state. If invert is set
     * to true, then isActive() returns true when the button is up and false when the button is down. */
    public TriggerButton(Button.BoolLambda button, boolean invert) {
        this.button = button;
        this.invert = invert;
    }

    /** Only active on new presses. Button must be released and pressed again to
     * become active again. */
    @Override
    public boolean isActive() {
        boolean active = invert ^ button.get();
        if (active && !latched) {
            latched = true;
            return true;
        } else if (!active && latched) {
            latched = false;
            return false;
        } else {
            return false;
        }
    }

}
