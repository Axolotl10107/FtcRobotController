package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.hardwaredevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;

/** A blank implementation of {@link PixelArm} that does nothing.
 * Never returns null (returns blank objects instead where applicable), so hopefully no null pointers. */
public class DoubleArmBlank implements DoubleArm {

    @Override
    public void setPivotAngle(AngleUnit unit, double angle) {

    }

    @Override
    public void setPivotPower(double power) {

    }

    @Override
    public double getPivotPower() {
        return 0;
    }

    @Override
    public void setPivotVelocity(int velocity) {

    }

    @Override
    public double getPivotVelocity() {
        return 0;
    }

    @Override
    public int getPivotPosition() {
        return 0;
    }

    @Override
    public void setElevatorDistance(double distance) {

    }

    @Override
    public void setElevatorPower(double power) {

    }

    @Override
    public double getElevatorPower() {
        return 0;
    }

    @Override
    public void setElevatorVelocity(int velocity) {

    }

    @Override
    public double getElevatorVelocity() {
        return 0;
    }

    @Override
    public int getElevatorPosition() {
        return 0;
    }

    @Override
    public void update() {

    }

}
