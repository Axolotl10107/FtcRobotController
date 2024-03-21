package org.firstinspires.ftc.teamcode.fy23.processors;

import com.qualcomm.robotcore.util.Range;

/** Currently, this is a 1-Dimensional acceleration limiter. It's quite simple for now, but there are future plans to
 * make this work either in more dimensions, with more motors, or both. We're not sure yet. */
public class AccelLimiter {
    // Be consistent with your units! If maxAccel is in meters per second squared, pass in seconds to request().
    // If it's in, like, centimeters per millisecond squared, pass in CpM and milliseconds to request().

    private double maxAccel; // meters per second per second
    private double maxDeltaVEachLoop; // just to prevent big jumps (jerks) on long loops

    private double _lastTime;
//    private double _oldVel;
//    private double _oldDeltaV;

    private boolean initialized = false;

    /** maxAccel is the maximum acceleration (in any unit you want, but we usually use meters per second), and
     * maxDeltaVEachLoop is the maximum change in velocity each loop (prevents a sudden velocity change / jerk if a loop
     * takes too long) */
    public AccelLimiter(double maxAccel, double maxDeltaVEachLoop) {
        setParameters(maxAccel, maxDeltaVEachLoop);
    }

    public void setParameters(double maxAccel, double maxDeltaVEachLoop) {
        this.maxAccel = maxAccel;
        this.maxDeltaVEachLoop = maxDeltaVEachLoop;
    }

    /** Request the desired final velocity, and this will return the velocity that you can safely go now given the
     * parameters you entered into the constructor. */
    public double requestVel(double newVel, double currentVel, double currentTime) {
        return currentVel + requestDeltaVel(newVel - currentVel, currentTime);
    }

    /** Request the desired change in velocity, and this will return how much velocity you can safely add to your
     * current velocity given the parameters you entered into the constructor. */
    public double requestDeltaVel(double deltaVel, double currentTime) {
        if (initialized) {
            double loopTime = currentTime - _lastTime;
//            double requestedDeltaVThisLoop = newVel - currentVel;
            double requestedDeltaVThisLoop = deltaVel;
            double targetDeltaVThisLoop = maxAccel * loopTime;
            double safeDeltaVThisLoop = Math.min(targetDeltaVThisLoop, maxDeltaVEachLoop);
            double actualDeltaVThisLoop = Range.clip(requestedDeltaVThisLoop, -safeDeltaVThisLoop, safeDeltaVThisLoop);
            _lastTime = currentTime;
            return actualDeltaVThisLoop;
        } else {
            _lastTime = currentTime;
            initialized = true;
            return 0;
        }
    }

    /** How much distance is needed to stop from the given initial velocity at the maximum acceleration set for your
     * AccelLimiter instance? Resolution is an integer. Higher resolution values make the calculation take longer but
     * yield more accurate results. Use the "stoppingDistancePrinter" Unit Test to determine what resolution you need. */
    public double stoppingDistance(double currentVel, int resolution) {
        double timeStep = 1.0 / resolution;
        double currentTime = 0;
        double totalDistance = 0;
//        System.out.println("currentTime | currentVel | totalDistance");
        while (currentVel > 0.01) {
            currentVel = requestVel(0, currentVel, currentTime);
            totalDistance += currentVel / 1000 * (timeStep * 1000); // dimensional analysis - velocity to milliseconds to distance each iter
            // example: (5 meters / 1 second) * (1 second / 1000 ms) * ((0.1 seconds / 1 iteration) * (1000 ms / 1 second) = (0.5 meters / 1 iteration)
//            System.out.println(String.format("{%f} | {%f} | {%f}", currentTime, currentVel, totalDistance));
            currentTime += timeStep;
        }
        return totalDistance;
    }


// old stuff

//    public double requestVelocityAndReturnNewVelocity(double newVel, double currentVel, double currentTime) {
//        if (initialized) {
//            double loopTime = currentTime - _lastTime;
//            double requestedDeltaVThisLoop = newVel - _oldVel;
//            double targetDeltaVThisLoop = maxAccel * loopTime;
//            double safeDeltaVThisLoop = Math.min(targetDeltaVThisLoop, maxDeltaVEachLoop);
//            double actualDeltaVThisLoop = Range.clip(requestedDeltaVThisLoop, -safeDeltaVThisLoop, safeDeltaVThisLoop);
//            double returnVel = _oldVel + actualDeltaVThisLoop;
//            _oldVel = returnVel;
//            _lastTime = currentTime;
////            System.out.println(String.format("_oldVel: {%f}", _oldVel));
////            System.out.println(String.format("returnVel: {%f}", returnVel));
////            System.out.println(String.format("_lastTime: {%f}", _lastTime));
//            return returnVel;
//        } else {
//            System.out.println("Initializing");
//            _oldVel = currentVel;
//            _lastTime = currentTime;
//            initialized = true;
//            return _oldVel;
//        }
//    }

//    public double requestDeltaVel(double newVel, double currentVel, double currentTime) {
//        if (initialized) {
//            double loopTime = currentTime - _lastTime;
////            double requestedDeltaVThisLoop = newVel - _oldVel;
//            double requestedDeltaVThisLoop = newVel - currentVel;
//            double targetDeltaVThisLoop = maxAccel * loopTime;
//            double safeDeltaVThisLoop = Math.min(targetDeltaVThisLoop, maxDeltaVEachLoop);
//            double actualDeltaVThisLoop = Range.clip(requestedDeltaVThisLoop, -safeDeltaVThisLoop, safeDeltaVThisLoop);
//            _oldDeltaV = actualDeltaVThisLoop;
//            _lastTime = currentTime;
//            return actualDeltaVThisLoop;
//        } else {
////            _oldVel = currentVel;
//            _lastTime = currentTime;
//            initialized = true;
//            return 0;
//        }
//    }

}
