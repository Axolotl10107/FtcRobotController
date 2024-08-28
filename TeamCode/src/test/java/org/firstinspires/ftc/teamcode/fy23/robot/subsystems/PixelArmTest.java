package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.fy23.fakestuff.MockDcMotorEx;
import org.firstinspires.ftc.teamcode.fy23.fakestuff.MockDigitalDevice;
import org.firstinspires.ftc.teamcode.fy23.fakestuff.MockElapsedTime;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PixelArmImpl;
import org.firstinspires.ftc.teamcode.fy23.units.SimplePowerTpSConverter;
import org.junit.Assert;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

public class PixelArmTest {

    org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm.Parameters params = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm.Parameters(true);
    MockElapsedTime pivotStopwatch = new MockElapsedTime();
    MockElapsedTime elevatorStopwatch = new MockElapsedTime();
    MockDcMotorEx pivotMotor = new MockDcMotorEx(pivotStopwatch);
    MockDcMotorEx elevatorMotor = new MockDcMotorEx(elevatorStopwatch);

    void initializeParams() {
        params.pivotMotor = pivotMotor;
        params.elevatorMotor = elevatorMotor;
        params.pivotAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        params.pivotTicksPerDegree = 10; // TODO: not measured!!
        params.pivotUpperLimit = 2000; // TODO: not measured on real hardware!!
        params.pivotLowerLimit = 0; // TODO: not measured on real hardware!!
        params.pivotUpperLimitSwitch = new MockDigitalDevice(false); // not installed
        params.pivotLowerLimitSwitch = new MockDigitalDevice(true); // not installed
        params.elevatorAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        params.elevatorTicksPerMillimeter = 10; // TODO: not measured!!
        params.elevatorUpperLimit = 2500;
        params.elevatorLowerLimit = 0;
        params.elevatorUpperLimitSwitch = new MockDigitalDevice(false); // not installed
        params.elevatorLowerLimitSwitch = new MockDigitalDevice(true); // not installed
        params.stopwatch = new ElapsedTime();
    }

    @Test
    public void testPivotSoftLimits() {
        initializeParams();
        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm pixelArm = new PixelArmImpl(params);

        // lower limit (assuming we start at the lower limit)
        pixelArm.setPivotPower(-1);
        pixelArm.update();
        Assert.assertEquals(0, pivotMotor.getPower(), 0.01);

        // can still move upwards
        pixelArm.setPivotPower(1);
        pixelArm.update();
        pixelArm.update();
        pivotStopwatch.setNanos(TimeUnit.SECONDS.toNanos(1));
        pixelArm.update();
        pixelArm.update();
        Assert.assertEquals(1, pixelArm.getPivotPower(), 0.01);

        // upper limit
        pivotMotor.setPosition(params.pivotUpperLimit);
        pixelArm.update();
        Assert.assertEquals(0, pivotMotor.getPower(), 0.01);

        // can still move downwards
        pixelArm.setPivotPower(-1);
        pixelArm.update();
        Assert.assertEquals(1, pivotMotor.getPower(), 0.01);
    }

    @Test
    public void testElevatorSoftLimits() {

    }

    @Test
    public void testPivotHardLimits() {

    }

    @Test
    public void testElevatorHardLimits() {

    }

}
