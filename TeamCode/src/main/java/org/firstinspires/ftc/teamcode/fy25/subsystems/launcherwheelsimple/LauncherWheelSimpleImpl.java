package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple;

import static org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple.LauncherWheelSimpleImpl.LaunchWheelPIDF.d;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple.LauncherWheelSimpleImpl.LaunchWheelPIDF.f;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple.LauncherWheelSimpleImpl.LaunchWheelPIDF.i;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple.LauncherWheelSimpleImpl.LaunchWheelPIDF.p;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LauncherWheelSimpleImpl implements LauncherWheelSimple{
    DcMotorEx motor;
    double vel = 0;

    @Config
    public static class LaunchWheelPIDF {
        public static double p = 6;
        public static double i = 0;
        public static double d = 1;
        public static double f = 0.03;
    }


    public LauncherWheelSimpleImpl(Parameters parameters) {
        motor = parameters.motor;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(p, i, d, f);
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
