package org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;

/** Makes a physical gamepad button act as an axis. */
public class ButtonAsAxis implements Axis {

    private final Button button;
    private final double scalingFactor;

    /** Pass in an existing {@link Button} */
    public ButtonAsAxis(Button button) {
        this.button = button;
        this.scalingFactor = 1;
    }

    /** In practice, the scaling factor is the output value. */
    public ButtonAsAxis(Button button, double scalingFactor) {
        this.button = button;
        this.scalingFactor = scalingFactor;
    }

    /** If the button is pressed, report 1. Otherwise, report 0. */
    @Override
    public double value() {
        return (button.isActive() ? 1 : 0) * scalingFactor;
    }
}
