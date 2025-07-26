package org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Axis;

/** Use this to combine two axes that only run from 0 to 1 into one axis that runs from -1 to 1. */
public class MergedAxis implements Axis {

    private final Axis negative;
    private final Axis positive;

    /** Pass in two existing {@link Axis}.
     *  @param negative Represents the negative direction (-1 to 0).
     *  @param positive Represents the positive direction (0 to 1). */
    public MergedAxis(Axis negative, Axis positive) {
        this.negative = negative;
        this.positive = positive;
    }

    /** @return Returns positive.value() - negative.value(). */
    @Override
    public double value() {
        return positive.value() - negative.value();
    }
}
