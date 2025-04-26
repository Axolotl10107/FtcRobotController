package org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.buttons;

import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.Button;

/** Makes a physical gamepad axis act as a button. */
public class AxisAsButton implements Button {

    private Axis.DoubleLambda axis;
    private boolean invert = false;
    private double threshold;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * axis field:
     * new LinearAxis( () -{@literal >} gamepad.left_stick_x );
     * and also pass in an activation threshold along the range of the axis, after which the
     * "button" is considered active */
    public AxisAsButton(Axis.DoubleLambda axis, double threshold) {
        this.axis = axis;
        this.threshold = threshold;
    }

    /** Same as the other constructor, but allows for inverting the reported value */
    public AxisAsButton(Axis.DoubleLambda axis, double threshold, boolean invert) {
        this.axis = axis;
        this.invert = invert;
        this.threshold = threshold;
    }

    @Override
    public boolean isActive() {
        return invert ^ (axis.get() > threshold ? true : false);
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
