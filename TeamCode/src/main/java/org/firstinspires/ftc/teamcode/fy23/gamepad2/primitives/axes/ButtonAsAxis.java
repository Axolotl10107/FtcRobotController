package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Axis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Button;

/** Makes a physical gamepad button act as an axis. */
public class ButtonAsAxis implements Axis {

    private Button.BoolLambda button;
    private double scalingFactor = 1;

    /** Pass in a lambda expression that returns the value of a {@link com.qualcomm.robotcore.hardware.Gamepad}
     * button field:
     * new ToggleButton( () -{@literal >} gamepad.x ); */
    public ButtonAsAxis(Button.BoolLambda button) {
        this.button = button;
    }

    /** In practice, the scaling factor is the output value. */
    public ButtonAsAxis(Button.BoolLambda button, double scalingFactor) {
        this.button = button;
        this.scalingFactor = scalingFactor;
    }

    @Override
    /** If the button is pressed, report 1. Otherwise, report 0. If the "axis" is inverted, report
     * -1 when pressed..*/
    public double value() {
        return (button.get() ? 1 : 0) * scalingFactor;
    }
}
