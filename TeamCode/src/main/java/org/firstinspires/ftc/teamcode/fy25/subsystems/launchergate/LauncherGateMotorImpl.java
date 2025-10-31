package org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;

public class LauncherGateMotorImpl implements LauncherGate {
    DcMotorSimple motor;
    double gatePower;
    Telemetry telemetry;

    public LauncherGateMotorImpl(Parameters parameters) {
        motor = parameters.device;
        gatePower = parameters.power;
        telemetry = TelemetrySingleton.getInstance();
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
