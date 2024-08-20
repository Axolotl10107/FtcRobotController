package org.firstinspires.ftc.teamcode.fy23.processors;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** A suite of tools for controlling acceleration. Currently works in one dimension. Can limit acceleration (<i>Phase 1</i>),
 * calculate stopping distance (<i>Phase 2</i>), and ramp up and down to a target position (<i>Phase 3</i>).
 * Be consistent with your units! If maxAccel is in meters per second squared, for example, pass in seconds to request()
 * (the currentTime parameter).
 * <b>Follow the progress of this class on the task board, in the Filters column.</b> */
public class AccelLimiter {
    // Be consistent with your units! If maxAccel is in meters per second squared, pass in seconds to request().
    // If it's in, like, centimeters per millisecond squared, pass in CpM and milliseconds to request().

    private double maxAccel; // meters per second per second
    private double maxDeltaVEachLoop; // just to prevent big jumps (jerks) on long loops

    private double _lastTime;

    // persistent storage for rampAlongDistance
    private double maxVelocity;
    private double stoppingPoint;
    private double lastOutput;
    private ElapsedTime stopwatch;

    private boolean initialized = false;

    /** Create an AccelLimiter object. (You must do this - the methods are not static.) It will maintain its state until
     * it is reset (using the reset() method).
     * @param maxAccel The maximum acceleration (in any unit you want, but we usually use meters per second)
     * @param maxDeltaVEachLoop The maximum change in velocity each loop (prevents a sudden velocity change / jerk if a
     * loop takes too long) */
    public AccelLimiter(double maxAccel, double maxDeltaVEachLoop) {
        setParameters(maxAccel, maxDeltaVEachLoop);
    }

    /** Change the parameters of this AccelLimiter instance.
     * @param maxAccel The maximum acceleration (in any unit you want, but we usually use meters per second)
     * @param maxDeltaVEachLoop The maximum change in velocity each loop (prevents a sudden velocity change / jerk if a
     * loop takes too long) */
    public void setParameters(double maxAccel, double maxDeltaVEachLoop) {
        this.maxAccel = maxAccel;
        this.maxDeltaVEachLoop = maxDeltaVEachLoop;
    }

    /** Get the maximum acceleration. (You set this in the constructor.) */
    public double getMaxAccel() {
        return maxAccel;
    }

    /** Get the maximum change in velocity each loop. (You set this in the constructor.) */
    public double getMaxDeltaVEachLoop() {
        return maxDeltaVEachLoop;
    }

    /** Request the desired final velocity, and this will return the velocity that you can safely go now given the
     * parameters you entered into the constructor.
     * (Will always return 0 while it initializes the first time it's called since this AccelLimiter was created or
     * reset)
     * @param newVel The velocity you want to go
     * @param currentVel The velocity you are currently going
     * @param currentTime The current time, in the same unit as the time component of your velocity (ex. if velocity is
     * in meters per second, then currentTime should be in seconds) */
    public double requestVel(double newVel, double currentVel, double currentTime) {
        return currentVel + requestDeltaVel(newVel - currentVel, currentTime);
    }

    /** Request the desired change in velocity, and this will return how much velocity you can safely add to your
     * current velocity given the parameters you entered into the constructor.
     * (Will always return 0 while it initializes the first time it's called since this AccelLimiter was created or
     * reset)
     * @param deltaVel How much you want to change your velocity
     * @param currentTime The current time, in the same unit as the time component of your velocity (ex. if velocity is
     * in meters per second, then currentTime should be in seconds) */
    public double requestDeltaVel(double deltaVel, double currentTime) {
        double requestedDeltaVThisLoop = deltaVel;
        double actualDeltaVThisLoop = makeDeltaVSafe(requestedDeltaVThisLoop, currentTime);
        _lastTime = currentTime;
        return actualDeltaVThisLoop;
    }

    // Phase 1 in smaller components
    /** How much can you change your velocity this loop given the maximum acceleration and how long it's been since last
     * time we calculated this? (Will always return 0 while it initializes the first time it's called since this
     * AccelLimiter was created or reset)
     * @param currentTime The current time, in the same unit as everything else (ex. if the maximum acceleration is in
     * meters per second, then currentTime should be in seconds) */
    public double getMaxDeltaVThisLoop(double currentTime) {
        if (initialized) {
            double loopTime = currentTime - _lastTime;
            double maxAccelThisLoop = maxAccel * loopTime;
            _lastTime = currentTime;
            return Math.min(maxAccelThisLoop, maxDeltaVEachLoop);
        } else {
            _lastTime = currentTime;
            initialized = true;
            return 0;
        }
    }

    /** Limit a given change in velocity to the maximum acceleration, with consideration to how long it's been since
     * last time we calculated this.
     * @param deltaV How much you want to change your velocity
     * @param currentTime The current time, in the same unit as everything else (ex. if the maximum acceleration is in
     * meters per second, then currentTime should be in seconds) */
    public double makeDeltaVSafe(double deltaV, double currentTime) {
        double safeDeltaV = getMaxDeltaVThisLoop(currentTime);
        return Range.clip(deltaV, -safeDeltaV, safeDeltaV);
    }

    /** Request many changes in velocity, and limit them while maintaining the proportions between them. (Works
     * similarly to {@link org.firstinspires.ftc.teamcode.fy23.units.DTS}.normalize().)
     * (Will always return 0 while it initializes the first time it's called since this AccelLimiter was created or
     * reset)
     * @param deltaVelList A List{@literal <}Double{@literal >} of all the changes in velocity you are requesting
     * @param currentTime The current time, in the same unit as everything else (ex. if the maximum acceleration is in
     * meters per second, then currentTime should be in seconds) */
    public List<Double> requestDeltaVelOnN(List<Double> deltaVelList, double currentTime) {
        double maxDeltaV = getMaxDeltaVThisLoop(currentTime);
        double biggestDeltaV = Math.abs(Collections.max(deltaVelList));
        // took the absolute value for the condition and scalingFactor calculation later
        List<Double> returnList = new ArrayList<Double>();
        if (biggestDeltaV > maxDeltaV) {
            double scalingFactor = maxDeltaV / biggestDeltaV;
            for (double item : deltaVelList) {
                returnList.add(item * scalingFactor);
            }
            return returnList;
        } else {
            return deltaVelList;
        }
    }

    /** Call this when you're done with your task and want to use this object for something else. */
    public void reset() {
        initialized = false;
        lastOutput = 0;
    }

    /** How much distance is needed to stop from the given initial velocity at the maximum acceleration set for your
     * AccelLimiter instance? Note that negative velocities still return positive stopping distances.
     * This method sums the area under the curve of the acceleration (the integral). This is not necessary for the
     * linear acceleration that AccelLimiter does exclusively at the moment (see the simpleStoppingDistance method), but
     * it may become useful with new functionality in the future.
     * @param initialVel How fast you are going at the moment you start ramping down
     * @param resolution Higher resolution values make the calculation take longer but yield more accurate results. Use
     * the "stoppingDistancePrinter" Unit Test to determine what resolution you need. */
    // TODO: Acceleration is linear, so you can just find the area of a right triangle...
    public double stoppingDistance(double initialVel, int resolution) {
        initialVel = Math.abs(initialVel);
        double timeStep = 1.0 / resolution;
        double currentTime = 0.0;
        double totalDistance = 0.0;
        while (initialVel > 0.01) {
            initialVel = requestVel(0, initialVel, currentTime);
            totalDistance += initialVel / 1000 * (timeStep * 1000); // dimensional analysis - velocity to milliseconds to distance each iter
            // example: (5 meters / 1 second) * (1 second / 1000 ms) * ((0.1 seconds / 1 iteration) * (1000 ms / 1 second) = (0.5 meters / 1 iteration)
//            System.out.println(String.format("{%f} | {%f} | {%f}", currentTime, currentVel, totalDistance));
            currentTime += timeStep;
        }
        return totalDistance;
    }

    /** A much faster stopping distance calculation that can be used for linear acceleration (which is all AccelLimiter
     * supports at the moment anyway). */
    public double simpleStoppingDistance(double initialVel) {
        initialVel = Math.abs(initialVel);
        double stoppingTime = initialVel / maxAccel;
        // For a triangle, A = 0.5 * b * h. Here, time * initialVel would create a rectangle, but the line of
        // acceleration cuts it in half, hence the 0.5 factor.
        return 0.5 * stoppingTime * initialVel;
    }

    /** Sets up ramping up to maxVelocity and back down along the specified distance. Will also reset this instance!
     * @param targetPos The position you want to end at (in whatever units you want, as long as you are consistent)
     * @param maxVelocity The top speed along that distance */
    public void setupRampToTarget(double targetPos, double maxVelocity) {
        setupRampToTarget(targetPos, maxVelocity, new ElapsedTime());
    }

    /** Used for dependency injection in UnitTests
     * @param targetPos The position you want to end at (in whatever units you want, as long as you are consistent)
     * @param maxVelocity The top speed along that distance
     * @param stopwatch Pass in a MockElapsedTime to control time in UnitTests. */
    public void setupRampToTarget(double targetPos, double maxVelocity, ElapsedTime stopwatch) {
        this.maxVelocity = maxVelocity;
        double stoppingDistance = stoppingDistance(maxVelocity, 10000);
        stoppingPoint = targetPos - stoppingDistance;
        System.out.println(stoppingDistance);
        System.out.println(stoppingPoint);
        this.stopwatch = stopwatch;
        reset();
    }

    /** Run this every loop. Takes the current position and returns the current velocity along the ramp.
     * @param currentPos Where are you currently? Should be in the same distance unit as the maximum acceleration. */
    public double updateRampToTarget(double currentPos) {
        if (currentPos < stoppingPoint) {
            lastOutput = requestVel(maxVelocity, lastOutput, stopwatch.seconds());
        } else {
            lastOutput = requestVel(0, lastOutput, stopwatch.seconds());
        }
        return lastOutput;
    }
}
