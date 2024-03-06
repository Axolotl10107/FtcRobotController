package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Axis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Button;

/** Makes a physical gamepad button act as an axis. */
public class ButtonAsAxis implements Axis {

    private Button.BoolLambda button;
    private int invert = 1;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * button field:
     * new ToggleButton( () -> gamepad.x ); */
    public ButtonAsAxis(Button.BoolLambda button) {
        this.button = button;
    }

    /** Same as the other constructor, but allows for inverting (negating) the reported value of the
     * "axis". */
    public ButtonAsAxis(Button.BoolLambda button, boolean invert) {
        this.button = button;
        this.invert = invert ? -1 : 1;
    }

    @Override
    /** If the button is pressed, report 1. Otherwise, report 0. If the "axis" is inverted, report
     * -1 when pressed..*/
    public double value() {
        return (button.get() ? 1 : 0) * invert;
    }
}
