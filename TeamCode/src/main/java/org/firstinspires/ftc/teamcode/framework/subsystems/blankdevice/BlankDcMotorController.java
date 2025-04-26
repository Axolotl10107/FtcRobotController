package org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class BlankDcMotorController extends BlankHardwareDevice implements DcMotorControllerEx {

    @Override
    public void setMotorType(int motor, MotorConfigurationType motorType) {

    }

    @Override
    public MotorConfigurationType getMotorType(int motor) {
        return MotorConfigurationType.getUnspecifiedMotorType();
    }

    @Override
    public void setMotorMode(int motor, DcMotor.RunMode mode) {

    }

    @Override
    public DcMotor.RunMode getMotorMode(int motor) {
        return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
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
    public void setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int motor) {
        return DcMotor.ZeroPowerBehavior.FLOAT;
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
    public void setMotorEnable(int motor) {

    }

    @Override
    public void setMotorDisable(int motor) {

    }

    @Override
    public boolean isMotorEnabled(int motor) {
        return false;
    }

    @Override
    public void setMotorVelocity(int motor, double ticksPerSecond) {

    }

    @Override
    public void setMotorVelocity(int motor, double angularRate, AngleUnit unit) {

    }

    @Override
    public double getMotorVelocity(int motor) {
        return 0;
    }

    @Override
    public double getMotorVelocity(int motor, AngleUnit unit) {
        return 0;
    }

    @Override
    public void setPIDCoefficients(int motor, DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {

    }

    @Override
    public void setPIDFCoefficients(int motor, DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {

    }

    @Override
    public PIDCoefficients getPIDCoefficients(int motor, DcMotor.RunMode mode) {
        return new PIDCoefficients(0, 0, 0);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(int motor, DcMotor.RunMode mode) {
        return new PIDFCoefficients(0, 0, 0, 0);
    }

    @Override
    public void setMotorTargetPosition(int motor, int position, int tolerance) {

    }

    @Override
    public double getMotorCurrent(int motor, CurrentUnit unit) {
        return 0;
    }

    @Override
    public double getMotorCurrentAlert(int motor, CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setMotorCurrentAlert(int motor, double current, CurrentUnit unit) {

    }

    @Override
    public boolean isMotorOverCurrent(int motor) {
        return false;
    }
}
