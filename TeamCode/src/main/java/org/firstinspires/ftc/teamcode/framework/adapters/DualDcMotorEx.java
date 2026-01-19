package org.firstinspires.ftc.teamcode.framework.adapters;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Turns two DcMotorEx-i into a family, a body that moves as one. */
public class DualDcMotorEx implements DcMotorEx {

    DcMotorEx primary;
    DcMotorEx secondary;

    /**
     * Create a DualDcMotorEx to represent 2 physical DcMotorEx-i.
     * @param primary The primary motor. Values will be reported from this motor's perspective.
     * @param secondary The secondary motor.
     */
    public DualDcMotorEx(DcMotorEx primary, DcMotorEx secondary) {
        this.primary = primary;
        this.secondary = secondary;
    }

    @Override
    public void setMotorEnable() {
        primary.setMotorEnable();
        secondary.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        primary.setMotorDisable();
        secondary.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return primary.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        primary.setVelocity(angularRate);
        secondary.setVelocity(angularRate);
        //added the negative, delete if it breaks smth...it worked to rverse the wheel intake thing
        //now i cant push this again kuz it wants to update the emulator
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        primary.setVelocity(angularRate, unit);
        secondary.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return (primary.getVelocity() + secondary.getVelocity()) / 2;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return (primary.getVelocity(unit) + secondary.getVelocity(unit)) / 2;
    }

    @Deprecated
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        primary.setPIDCoefficients(mode, pidCoefficients);
        secondary.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        primary.setPIDFCoefficients(mode, pidfCoefficients);
        secondary.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        primary.setVelocityPIDFCoefficients(p, i, d, f);
        secondary.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        primary.setPositionPIDFCoefficients(p);
        secondary.setPositionPIDFCoefficients(p);
    }

    @Deprecated
    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return primary.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return primary.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        primary.setTargetPositionTolerance(tolerance);
        secondary.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return primary.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return (primary.getCurrent(unit) + secondary.getCurrent(unit)) / 2;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return primary.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        primary.setCurrentAlert(current, unit);
        secondary.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return primary.isOverCurrent() || secondary.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return primary.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        primary.setMotorType(motorType);
        secondary.setMotorType(motorType);
    }

    /** Returns null! Don't use this! */
    @Override
    public DcMotorController getController() {
        // This is to fail fast if someone ends up using this.
        // It's not safe for something to interact more directly with just one motor.
        return null;
    }

    /** Returns -1. */
    @Override
    public int getPortNumber() {
        return -1;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        primary.setZeroPowerBehavior(zeroPowerBehavior);
        secondary.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return primary.getZeroPowerBehavior();
    }

    @Deprecated
    @Override
    public void setPowerFloat() {
        primary.setPowerFloat();
        secondary.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return primary.getPowerFloat() || secondary.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        primary.setTargetPosition(position);
        secondary.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return primary.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return primary.isBusy() || secondary.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return (primary.getCurrentPosition() + secondary.getCurrentPosition()) / 2;
    }

    @Override
    public void setMode(RunMode mode) {
        primary.setMode(mode);
        secondary.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return primary.getMode();
    }

    /** If the direction to set is different from the primary's current direction, this sets both
     * motors to the opposite of their current directions. */
    @Override
    public void setDirection(Direction direction) {
        if (direction != primary.getDirection()) {
            if (primary.getDirection() == DcMotorEx.Direction.FORWARD) {
                primary.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                primary.setDirection(DcMotorEx.Direction.FORWARD);
            }
            if (secondary.getDirection() == DcMotorEx.Direction.FORWARD) {
                secondary.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                secondary.setDirection(DcMotorEx.Direction.FORWARD);
            }
        }
    }

    /** Returns the direction of the primary motor. */
    @Override
    public Direction getDirection() {
        return primary.getDirection();
    }

    @Override
    public void setPower(double power) {
        primary.setPower(power);
        secondary.setPower(power);
    }

    @Override
    public double getPower() {
        return (primary.getPower() + secondary.getPower()) / 2;
    }

    @Override
    public Manufacturer getManufacturer() {
        return primary.getManufacturer();
    }

    /** Returns "Dual" followed by the primary's device name then the secondary's device name. */
    @Override
    public String getDeviceName() {
        return "Dual " + primary.getDeviceName() + " " + secondary.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return primary.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return primary.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        primary.resetDeviceConfigurationForOpMode();
        secondary.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        primary.close();
        secondary.close();
    }
}
