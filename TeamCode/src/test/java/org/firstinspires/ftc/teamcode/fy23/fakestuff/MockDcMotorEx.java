package org.firstinspires.ftc.teamcode.fy23.fakestuff;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;
import java.util.Map;

public class MockDcMotorEx implements DcMotorEx {

    private boolean motorEnabled = true;

    private double velocityTicks = 0;
    private double velocityRadians = 0;
    private double maxVelocityTicks = 150;
    private double ticksPerRotation = 537.7;
    private double maxVelocityRadians = (maxVelocityTicks / ticksPerRotation) * (2 * Math.PI);

    private double power;
    private int position;
    private int targetPosition;
    private int targetPositionTolerance;

    private Direction direction;
    private RunMode runMode = RunMode.RUN_WITHOUT_ENCODER;
    private MotorConfigurationType motorType;
    private int portNumber = -1;
    private ZeroPowerBehavior zeroPowerBehavior;
    private Map<RunMode, PIDCoefficients> pidCoefficientsMap = new HashMap<>();
    private Map<RunMode, PIDFCoefficients> pidfCoefficientsMap = new HashMap<>();

    private ElapsedTime elapsedTime;
    private double lastTime = 0;



    public MockDcMotorEx(ElapsedTime elapsedTime) {
        direction = Direction.FORWARD;
        this.elapsedTime = elapsedTime;
    }

    public MockDcMotorEx(Direction direction, ElapsedTime elapsedTime) {
        this.direction = direction;
        this.elapsedTime = elapsedTime;
    }

    public MockDcMotorEx(Direction direction, @NonNull MotorConfigurationType motorType, ElapsedTime elapsedTime) {
        this.direction = direction;
        this.motorType = motorType;
        this.elapsedTime = elapsedTime;
    }



    @Override
    public void setMotorEnable() {
        motorEnabled = true;
    }

    @Override
    public void setMotorDisable() {
        motorEnabled = false;
    }

    @Override
    public boolean isMotorEnabled() {
        return motorEnabled;
    }

    @Override
    public void setVelocity(double angularRate) {
        // in the future, might make an object that follows a curve of power to velocity gathered
        // from a real motor (a graph/curve/table/line of y, velocity, at each point x, power)
        velocityTicks = Range.clip(angularRate, -maxVelocityTicks, maxVelocityTicks);
        velocityRadians = (velocityTicks / ticksPerRotation) * (2 * Math.PI);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        velocityRadians = Range.clip(AngleUnit.RADIANS.fromUnit(unit, angularRate), -maxVelocityRadians, maxVelocityRadians);
        velocityTicks = (velocityRadians / (2 * Math.PI)) * ticksPerRotation;
    }

    @Override
    public double getVelocity() {
        return velocityRadians;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return unit.fromUnit(AngleUnit.RADIANS, velocityRadians);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        pidCoefficientsMap.put(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        pidfCoefficientsMap.put(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF));
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, 0, 0, 0, MotorControlAlgorithm.PIDF));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return pidCoefficientsMap.get(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return pidfCoefficientsMap.get(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        targetPositionTolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return targetPositionTolerance;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean isOverCurrent() {
        throw new UnsupportedOperationException();
    }

    // DcMotor
    @Override
    public MotorConfigurationType getMotorType() {
        return motorType;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        this.motorType = motorType;
    }

    @Override
    public DcMotorController getController() {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getPortNumber() {
        return portNumber;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    @Override
    public void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0.0);
    }

    @Override
    public boolean getPowerFloat() {
        return getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && getPower() == 0.0;
    }

    @Override
    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public boolean isBusy() {
        return position != targetPosition;
    }

    @Override
    public int getCurrentPosition() {
        reCalculate();
        return position;
    }

    @Override
    public void setMode(RunMode mode) {
        runMode = mode;
    }

    @Override
    public RunMode getMode() {
        return runMode;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPower(double power) {
        this.power = Range.clip(power, -1, 1);
    }

    @Override
    public double getPower() {
        return power;
    }

    // HardwareDevice
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "Mockery 0 RPM motor";
    }

    @Override
    public String getConnectionInfo() {
        return "This motor shares a connection with its friends.";
    }

    @Override
    public int getVersion() {
        return 51825;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        setDirection(Direction.FORWARD);
    }

    @Override
    public void close() {
        setPowerFloat();
    }


    // Methods for tests
    public void setMaxVelocityTicks(double maxVelocityTicks) {
        this.maxVelocityTicks = maxVelocityTicks;
    }

    public void setTicksPerRotation(double ticksPerRotation) {
        this.ticksPerRotation = ticksPerRotation;
    }

    private void reCalculate() {
        double deltaTime = elapsedTime.seconds() - lastTime;
        int deltaPosition = (int) Math.round(velocityTicks * deltaTime);
        position += deltaPosition;
    }
}
