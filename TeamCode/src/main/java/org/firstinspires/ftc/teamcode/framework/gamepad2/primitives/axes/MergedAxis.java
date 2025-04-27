package org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.axes;

import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.Axis;

/** Use this to combine two axes that only run from 0 to 1 into one axis that runs from -1 to 1. */
public class MergedAxis implements Axis {

    private final Axis axis1;
    private final Axis axis2;

    /** @param axis1 Represents the negative direction (-1 to 0).
     *  @param axis2 Represents the positive direction (0 to 1). */
    public MergedAxis(Axis axis1, Axis axis2) {
        this.axis1 = axis1;
        this.axis2 = axis2;
    }

    /** @return Returns axis2.value() - axis1.value(). */
    @Override
    public double value() {
        return axis2.value() - axis1.value();
    }
}
