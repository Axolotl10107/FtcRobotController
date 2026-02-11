package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LauncherWheelSimpleImpl implements LauncherWheelSimple{
    DcMotorEx motor;
    double vel = 0;


    public LauncherWheelSimpleImpl(Parameters parameters) {
        motor = parameters.motor;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(6, 0, 1, 0.03);
    }
    @Override
    public void spinUp(double velocity) {
        this.vel = velocity;
    }

    @Override
    public void spinDown() {
        vel = 0;
    }

    @Override
    public void update() {
        motor.setVelocity(vel);
    }
}
