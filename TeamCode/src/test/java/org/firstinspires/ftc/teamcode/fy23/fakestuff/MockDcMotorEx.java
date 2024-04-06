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

/** This appears as a normal {@link DcMotorEx} to your program, and it even updates the position for
 * you over time based on the set velocity and can determine the velocity the motor is running at
 * based on the set power (approximated as a percentage of the maximum velocity). It also handles
 * {@link com.qualcomm.robotcore.hardware.DcMotorEx.RunMode}.RUN_TO_POSITION mode, running at the set
 * power until the position is reached. Remember that this simulated class is perfect, without the
 * natural variance of a real motor. Have your test put this "device" into the
 * {@link com.qualcomm.robotcore.hardware.HardwareMap} with the correct name so that the test
 * subject can get it. */
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
    /** Unsupported Operation */
    public double getCurrent(CurrentUnit unit) {
        throw new UnsupportedOperationException();
    }

    @Override
    /** Unsupported Operation */
    public double getCurrentAlert(CurrentUnit unit) {
        throw new UnsupportedOperationException();
    }

    @Override
    /** Unsupported Operation */
    public void setCurrentAlert(double current, CurrentUnit unit) {
        throw new UnsupportedOperationException();
    }

    @Override
    /** Unsupported Operation */
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
    /** Unsupported Operation */
    public DcMotorController getController() {
        throw new UnsupportedOperationException();
    }

    @Override
    /** Should return -1, which is the SDK's default before it gets set */
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
    /** The name of this method is confusing. It simply sets the power to 0 and sets the {@link com.qualcomm.robotcore.hardware.DcMotorEx.ZeroPowerBehavior}
     * to FLOAT. This completely removes power from the motor. */
    public void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0.0);
    }

    @Override
    /** Returns if the motor is simply floating, completely unpowered. Specifically, returns if the
     * power is set to 0 and the {@link com.qualcomm.robotcore.hardware.DcMotorEx.ZeroPowerBehavior}
     * is set to FLOAT. */
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
    /** This implementation, and seemingly the SDK's, returns whether or not the current position is
     * the target position (the target has been reached, we're not still moving towards it). */
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
        velocityTicks = maxVelocityTicks * power;
        velocityRadians = maxVelocityRadians * power;
    }

    @Override
    public double getPower() {
        return power;
    }

    // HardwareDevice
    @Override
    /** Returns {@link com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer}.UNKNOWN */
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    /** Returns "Mockery 0 RPM motor" */
    public String getDeviceName() {
        return "Mockery 0 RPM motor";
    }

    @Override
    /** Returns "This motor shares a connection with its friends." */
    public String getConnectionInfo() {
        return "This motor shares a connection with its friends.";
    }

    @Override
    /** Returns 51825 */
    public int getVersion() {
        return 51825;
    }

    @Override
    /** Sets the {@link com.qualcomm.robotcore.hardware.DcMotorEx.Direction} to FORWARD.
     * (matches the original implementation) */
    public void resetDeviceConfigurationForOpMode() {
        setDirection(Direction.FORWARD);
    }

    @Override
    /** Calls setPowerFloat() (matches the original implementation) */
    public void close() {
        setPowerFloat();
    }


    // Methods for tests
    /** For use by your test - sets the maximum velocity of the motor in ticks per second */
    public void setMaxVelocityTicks(double maxVelocityTicks) {
        this.maxVelocityTicks = maxVelocityTicks;
    }

    /** For use by your test - sets the ticks counted by the encoder on each rotation of the motor */
    public void setTicksPerRotation(double ticksPerRotation) {
        this.ticksPerRotation = ticksPerRotation;
    }

    private void reCalculate() {
        if (runMode == RunMode.RUN_TO_POSITION && position >= targetPosition) {
            position = targetPosition;
            lastTime = elapsedTime.seconds();
        } else {
            double currentTime = elapsedTime.seconds();
            double deltaTime = currentTime - lastTime;
            int deltaPosition = (int) Math.round(velocityTicks * deltaTime);
            position += deltaPosition;
            lastTime = currentTime;
        }
    }
}
