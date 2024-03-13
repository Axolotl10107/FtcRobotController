package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

// Remember: you can just make something that works for now, and that is actually advisable.
// Make something that works first, *then* worry about architecting it once you actually know what
// pieces are involved and how they can reasonably be split up while still interacting well with
// each other.

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;

public class PixelArmImpl implements PixelArm {

    public PixelArmImpl(PixelArm.Parameters parameters, HardwareMap hardwareMap) {

    }

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
