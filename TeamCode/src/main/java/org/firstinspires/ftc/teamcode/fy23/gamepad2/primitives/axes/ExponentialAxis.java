package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Axis;

/** Represents an axis on a {@link com.qualcomm.robotcore.hardware.Gamepad}, including sticks and
 * triggers, and reports the value taken to the power specified in the constructor (this creates a
 * simple response curve - find out what it is by graphing y=x^[power] - y=x^2, for example)*/
public class ExponentialAxis implements Axis {

    private DoubleLambda axis;
    private int invert = 1;
    private double power = 1;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * axis field:
     * new ExponentialAxis( () -{@literal >} gamepad.left_stick_x );
     * and this.value() will report the value of that axis taken to the specified power. */
    public ExponentialAxis(DoubleLambda axis, double power) {
        this.axis = axis;
        this.power = power;
    }

    /** Same as the other constructor, but allows for inverting the reported value */
    public ExponentialAxis(DoubleLambda axis, boolean invert, double power) {
        this(axis, power);
        this.invert = invert ? -1 : 1;
    }

    @Override
    public double value() {
        return Math.pow(axis.get(), power) * invert;
    }
}
