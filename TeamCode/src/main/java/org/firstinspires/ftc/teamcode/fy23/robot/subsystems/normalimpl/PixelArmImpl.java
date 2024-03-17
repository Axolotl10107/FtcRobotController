package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

// Remember: you can just make something that works for now, and that is actually advisable.
// Make something that works first, *then* worry about architecting it once you actually know what
// pieces are involved and how they can reasonably be split up while still interacting well with
// each other.

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;

/** A normal implementation of {@link PixelArm} featuring acceleration limiting. */
public class PixelArmImpl implements PixelArm {

    private DcMotorEx pivotMotor;
    private DcMotorEx elevatorMotor;

    private AccelLimiter pivotAccelLimiter;
    private AccelLimiter elevatorAccelLimiter;

    private ElapsedTime stopwatch;

    public PixelArmImpl(PixelArm.Parameters parameters, HardwareMap hardwareMap, ElapsedTime stopwatch) {
        pivotMotor = hardwareMap.get(DcMotorEx.class, parameters.pivotMotorName);
        elevatorMotor = hardwareMap.get(DcMotorEx.class, parameters.elevatorMotorName);

        /** power per second */
        pivotAccelLimiter = new AccelLimiter(parameters.maxPivotAccel, parameters.maxPivotDeltaVEachLoop);
        /** power per second */
        elevatorAccelLimiter = new AccelLimiter(parameters.maxElevatorAccel, parameters.maxElevatorDeltaVEachLoop);

        this.stopwatch = stopwatch;
    }

    @Override
    public void setPivotPower(double power) {
        double applyPower = pivotAccelLimiter.requestVel(power, getPivotPower(), stopwatch.seconds());
        pivotMotor.setPower(applyPower);
    }

    @Override
    public double getPivotPower() {
        return pivotMotor.getPower();
    }

    @Override
    public void setElevatorPower(double power) {
        double applyPower = elevatorAccelLimiter.requestVel(power, getElevatorPower(), stopwatch.seconds());
        elevatorMotor.setPower(applyPower);
    }

    @Override
    public double getElevatorPower() {
        return elevatorMotor.getPower();
    }

    @Override
    public void update() {

    }

}
