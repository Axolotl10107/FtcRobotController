package org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;

/** Makes a physical gamepad button act as an axis. */
public class ButtonAsAxis implements Axis {

    private final Button.BoolLambda button;
    private final double scalingFactor;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * button field:
     * new ToggleButton( () -{@literal >} gamepad.x ); */
    public ButtonAsAxis(Button.BoolLambda button) {
        this(button, 1);
    }

    /** In practice, the scaling factor is the output value. */
    public ButtonAsAxis(Button.BoolLambda button, double scalingFactor) {
        this.button = button;
        this.scalingFactor = scalingFactor;
    }

    /** If the button is pressed, report 1. Otherwise, report 0. If the "axis" is inverted, report
     * -1 when pressed. */
    @Override
    public double value() {
        return (button.get() ? 1 : 0) * scalingFactor;
    }
}
