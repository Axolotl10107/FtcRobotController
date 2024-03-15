package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;

/** A blank implementation of {@link PixelArm} that does nothing. */
public class PixelArmBlank implements PixelArm {

    @Override
    public void setPivotPower(double power) {

    }

    @Override
    public double getPivotPower() {
        return 0;
    }

    @Override
    public void setElevatorPower(double power) {

    }

    @Override
    public double getElevatorPower() {
        return 0;
    }
}
