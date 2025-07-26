package org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;

/** Makes a physical gamepad axis act as a button. */
public class AxisAsButton implements Button {

    private final Axis axis;
    private final boolean invert;
    private final double threshold;

    /** Pass in an existing {@link Axis}
     * and also pass in an activation threshold along the range of the axis, after which the
     * "button" is considered active */
    public AxisAsButton(Axis axis, double threshold) {
        this(axis, threshold, false);
    }

    /** Same as the other constructor, but allows for inverting the reported value */
    public AxisAsButton(Axis axis, double threshold, boolean invert) {
        this.axis = axis;
        this.invert = invert;
        this.threshold = threshold;
    }

    /** Active when the axis value is past the threshold. */
    @Override
    public boolean isActive() {
        return invert ^ (axis.value() > threshold);
        /*
        XOR ( ^ ) - "exclusive or" - either A or B, but not both
        invert | button.get() | result
        0 | 0 | 0
        0 | 1 | 1
        1 | 0 | 1
        1 | 1 | 0
        */
    }
}
