package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Axis;

/** Represents an axis on a gamepad, including sticks or triggers, and directly reports the value. */
public class LinearAxis implements Axis {

    private DoubleLambda axis;
    private int invert = 1;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * axis field:
     * new LinearAxis( () -> gamepad.left_stick_x ); */
    public LinearAxis(DoubleLambda axis) {
        this.axis = axis;
    }

    /** Same as the other constructor, but allows for inverting the reported value */
    public LinearAxis(DoubleLambda axis, boolean invert) {
        this.axis = axis;
        this.invert = invert ? -1 : 1;
    }

    @Override
    /** Reports the value of the axis */
    public double value() {
        return axis.get() * invert;
    }
}
