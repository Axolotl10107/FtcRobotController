package org.firstinspires.ftc.teamcode.fy23.robot.processors;

import org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts;

/** A simple PID algorithm that allows its constants to be changed on the fly. Useful for tuning
 * them in a TeleOp to instantly see the results of changes. It can also import a {@link org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts}. */
public class TunablePID {

    private double proportional;
    private double integral = 0;
    private double integralMultiplier;
    private double derivative = 0;
    private double derivativeMultiplier;

    public TunablePID(double p, double im, double dm) {
        proportional = p;
        integralMultiplier = im;
        //TODO: GM0 recommends multiplying the integral by the time the last loop took to complete
        //so that a consistent amount is added each time. I would probably also need a multiplier on
        //that time, though, and I don't feel like tuning that right now.
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


    /** for telemetry */
    public double getIntegral() {
        return integral;
    }

    /** Resets the integral to 0 */
    public void clearIntegral() {
        integral = 0;
    }

    public void setIntegralMultiplier(double arg) {
        integralMultiplier = arg;
    }

    public double getIntegralMultiplier() {
        return integralMultiplier;
    }

    /** for telemetry */
    public double getDerivative() {
        return derivative;
    }

    /** Resets the derivative to 0 */
    public void clearDerivative() {
        derivative = 0;
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

    /** Given the "error" (distance from your target) and considering the currently set PID constants,
     * return the power suggested by the PID algorithm to progress toward the target (or, rather, an error of 0). */
    public double getCorrectionPower(double error, double lastError) {
        updateIntegral(error);
        updateDerivative(error, lastError);
        double finalp = proportional * error;
        double finali = integralMultiplier * integral;
        double finald = derivativeMultiplier * derivative;
        double gcd = Math.max((finalp + finali + finald), 1);
        return finalp/gcd + finali/gcd + finald/gcd;
    }
}
