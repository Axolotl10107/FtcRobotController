package org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;

/** Makes two physical gamepad buttons act as an axis (one is positive, one is negative). */
public class TwoButtonsAsAxis implements Axis {

    private final Button.BoolLambda button1;
    private final Button.BoolLambda button2;
    private double scalingFactor = 1;

    /** Pass in two lambda expressions that return the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * button field:
     * new ToggleButton( () -{@literal >} gamepad.x, () -{@literal >} gamepad.y, TwoButtonsAsAxis.Side.LEFT);
     * @param button1 Represents the negative direction.
     * @param button2 Represents the positive direction. */
    public TwoButtonsAsAxis(Button.BoolLambda button1, Button.BoolLambda button2) {
        this.button1 = button1;
        this.button2 = button2;
    }

    /** In practice, the scaling factor is the output value. */
    public TwoButtonsAsAxis(Button.BoolLambda button1, Button.BoolLambda button2, double scalingFactor) {
        this(button1, button2);
        this.scalingFactor = scalingFactor;
    }

    /** If the button is pressed, report 1. Otherwise, report 0. If the "axis" is inverted, report
     * -1 when pressed. */
    @Override
    public double value() {
        double button1value = button1.get() ? scalingFactor : 0;
        double button2value = button2.get() ? scalingFactor : 0;
        return button2value - button1value;
    }
}
