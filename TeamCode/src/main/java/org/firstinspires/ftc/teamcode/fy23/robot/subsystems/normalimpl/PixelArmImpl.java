package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

// Remember: you can just make something that works for now, and that is actually advisable.
// Make something that works first, *then* worry about architecting it once you actually know what
// pieces are involved and how they can reasonably be split up while still interacting well with
// each other.

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;

/** A normal implementation of {@link PixelArm}. */
public class PixelArmImpl implements PixelArm {

    private DcMotorEx pivotMotor;
    private DcMotorEx elevatorMotor;

    public PixelArmImpl(PixelArm.Parameters parameters, HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.get(DcMotorEx.class, parameters.pivotMotorName);
        elevatorMotor = hardwareMap.get(DcMotorEx.class, parameters.elevatorMotorName);
    }

    @Override
    public void setPivotPower(double power) {
        pivotMotor.setPower(power);
    }

    @Override
    public double getPivotPower() {
        return pivotMotor.getPower();
    }

    @Override
    public void setElevatorPower(double power) {
        elevatorMotor.setPower(0);
    }

    @Override
    public double getElevatorPower() {
        return elevatorMotor.getPower();
    }
}
