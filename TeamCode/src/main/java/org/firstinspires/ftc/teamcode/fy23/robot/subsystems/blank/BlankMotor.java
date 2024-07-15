package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** A blank implementation of DcMotorEx that does nothing.
 * Never returns null (returns blank objects instead where applicable), so hopefully no null pointers. */
public class BlankMotor implements DcMotorEx {
    @Override
    public void setMotorEnable() {

    }

    @Override
    public void setMotorDisable() {

    }

    @Override
    public boolean isMotorEnabled() {
        return false;
    }

    @Override
    public void setVelocity(double v) {

    }

    @Override
    public void setVelocity(double v, AngleUnit angleUnit) {

    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public double getVelocity(AngleUnit angleUnit) {
        return 0;
    }

    @Override
    public void setPIDCoefficients(RunMode runMode, PIDCoefficients pidCoefficients) {

    }

    @Override
    public void setPIDFCoefficients(RunMode runMode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {

    }

    @Override
    public void setVelocityPIDFCoefficients(double v, double v1, double v2, double v3) {

    }

    @Override
    public void setPositionPIDFCoefficients(double v) {

    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode runMode) {
        return new PIDCoefficients();
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode runMode) {
        return new PIDFCoefficients();
    }

    @Override
    public void setTargetPositionTolerance(int i) {

    }

    @Override
    public int getTargetPositionTolerance() {
        return 0;
    }

    @Override
    public double getCurrent(CurrentUnit currentUnit) {
        return 0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit currentUnit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double v, CurrentUnit currentUnit) {

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return new MotorConfigurationType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorConfigurationType) {

    }

    @Override
    public DcMotorController getController() {
        // Could easily replace this with a BlankMotorController - maybe a project for the future
        return new DcMotorController() {
            @Override
            public void setMotorType(int motor, MotorConfigurationType motorType) {

            }

            @Override
            public MotorConfigurationType getMotorType(int motor) {
                return new MotorConfigurationType();
            }

            @Override
            public void setMotorMode(int motor, RunMode mode) {

            }

            @Override
            public RunMode getMotorMode(int motor) {
                return RunMode.RUN_WITHOUT_ENCODER;
            }

            @Override
            public void setMotorPower(int motor, double power) {

            }

            @Override
            public double getMotorPower(int motor) {
                return 0;
            }

            @Override
            public boolean isBusy(int motor) {
                return false;
            }

            @Override
            public void setMotorZeroPowerBehavior(int motor, ZeroPowerBehavior zeroPowerBehavior) {

            }

            @Override
            public ZeroPowerBehavior getMotorZeroPowerBehavior(int motor) {
                return ZeroPowerBehavior.FLOAT;
            }

            @Override
            public boolean getMotorPowerFloat(int motor) {
                return false;
            }

            @Override
            public void setMotorTargetPosition(int motor, int position) {

            }

            @Override
            public int getMotorTargetPosition(int motor) {
                return 0;
            }

            @Override
            public int getMotorCurrentPosition(int motor) {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode(int motor) {

            }

            @Override
            public Manufacturer getManufacturer() {
                return Manufacturer.Unknown;
            }

            @Override
            public String getDeviceName() {
                return "DcMotorController inside of BlankMotor";
            }

            @Override
            public String getConnectionInfo() {
                return "Not Applicable";
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }
        };
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return ZeroPowerBehavior.FLOAT;
    }

    @Override
    public void setPowerFloat() {

    }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int i) {

    }

    @Override
    public int getTargetPosition() {
        return 0;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }

    @Override
    public void setMode(RunMode runMode) {

    }

    @Override
    public RunMode getMode() {
        return RunMode.RUN_WITHOUT_ENCODER;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return Direction.FORWARD;
    }

    @Override
    public void setPower(double v) {

    }

    @Override
    public double getPower() {
        return 0;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "BlankMotor";
    }

    @Override
    public String getConnectionInfo() {
        return "Not Applicable";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
