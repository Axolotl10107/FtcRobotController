package org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LauncherGateImpl implements LauncherGate {
    DcMotorSimple motor;
    double gatePower;

    public LauncherGateImpl(Parameters parameters) {
        motor = parameters.device;
        gatePower = parameters.power;
    }

    @Override
    public void open() {
        motor.setPower(gatePower);
    }

    @Override
    public void close() {
        motor.setPower(0);
    }

    @Override
    public boolean isOpen() {
        return (motor.getPower() > 0);
    }

    @Override
    public void setPower(double power) {
        gatePower = power;
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public void update() {

    }
}
