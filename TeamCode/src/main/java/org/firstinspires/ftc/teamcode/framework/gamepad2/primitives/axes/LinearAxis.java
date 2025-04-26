package org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.axes;

import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.Axis;

/** Represents an axis on a gamepad, including sticks or triggers, and directly reports the value. */
public class LinearAxis implements Axis {

    private DoubleLambda axis;
    private double scalingFactor;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * axis field:
     * new LinearAxis( () -{@literal >} gamepad.left_stick_x ); */
    public LinearAxis(DoubleLambda axis) {
        this(axis,1);
    }

    /** Specify a scaling factor (a number that the value should be multiplied by) */
    public LinearAxis(DoubleLambda axis, double scalingFactor) {
        this.axis = axis;
        this.scalingFactor = scalingFactor;
    }

    @Override
    /** Reports the value of the axis */
    public double value() {
        return axis.get() * scalingFactor;
    }
}
