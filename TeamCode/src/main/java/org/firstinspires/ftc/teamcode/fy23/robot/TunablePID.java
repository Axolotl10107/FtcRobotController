package org.firstinspires.ftc.teamcode.fy23.robot;

import org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts;

public class TunablePID {
    // I'm making this for the IMU stuff, so it works in floats because the IMU works in floats.

    private double proportional;
    private double integral = 0;
    private double integralMultiplier;
    private double derivative = 0;
    private double derivativeMultiplier;

    public TunablePID(double p, double im, double dm) {
        proportional = p;
        integralMultiplier = im;
        derivativeMultiplier = dm;
    }

    public TunablePID(PIDconsts pidConsts) { // function overloading
        proportional = pidConsts.p;
        integralMultiplier = pidConsts.im;
        derivativeMultiplier = pidConsts.dm;
    }


    public void setProportional(double arg) {
        proportional = arg;
    }

    public double getProportional() {
        return proportional;
    }


    public double getIntegral() {
        return integral;
    }

    public void setIntegralMultiplier(double arg) {
        integralMultiplier = arg;
    }

    public double getIntegralMultiplier() {
        return integralMultiplier;
    }


    public double getDerivative() {
        return derivative;
    }

    public void setDerivativeMultiplier(double arg) {
        derivativeMultiplier = arg;
    }

    public double getDerivativeMultiplier() {
        return derivativeMultiplier;
    }


    private void updateIntegral(double error) {
        integral += error;
    }

    private void updateDerivative(double error, double lastError) {
        derivative = error - lastError;
    }

    public double getCorrectionPower(double error, double lastError) {
        updateIntegral(error);
        updateDerivative(error, lastError);
        return
                (proportional * error)
                +
                (integralMultiplier * integral)
                +
                (derivativeMultiplier * derivative);
    }
}
