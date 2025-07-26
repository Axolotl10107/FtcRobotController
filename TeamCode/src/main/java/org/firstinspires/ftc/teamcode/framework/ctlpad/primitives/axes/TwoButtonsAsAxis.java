package org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;

/** Makes two physical gamepad buttons act as an axis (one is positive, one is negative). */
public class TwoButtonsAsAxis implements Axis {

    private final Button negative;
    private final Button positive;
    private double scalingFactor = 1;

    /** Pass in two existing {@link Button}.
     * @param negative Represents the negative direction.
     * @param positive Represents the positive direction. */
    public TwoButtonsAsAxis(Button negative, Button positive) {
        this.negative = negative;
        this.positive = positive;
    }

    /** In practice, the scaling factor is the output value. */
    public TwoButtonsAsAxis(Button negative, Button positive, double scalingFactor) {
        this(negative, positive);
        this.scalingFactor = scalingFactor;
    }

    /** If the button is pressed, report 1. Otherwise, report 0. If the "axis" is inverted, report
     * -1 when pressed. */
    @Override
    public double value() {
        double negativeValue = negative.isActive() ? scalingFactor : 0;
        double positiveValue = positive.isActive() ? scalingFactor : 0;
        return positiveValue - negativeValue;
    }
}
