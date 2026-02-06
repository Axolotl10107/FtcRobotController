package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LauncherWheelSimpleImpl implements LauncherWheelSimple{
    DcMotor motor;
    double power = 0;


    public LauncherWheelSimpleImpl(Parameters parameters) {
        motor = parameters.motor;

    }
    @Override
    public void spinUp(double power) {
        this.power = power;
    }

    @Override
    public void spinDown() {
        power = 0;
    }

    @Override
    public void update() {
        motor.setPower(power);
    }
}
