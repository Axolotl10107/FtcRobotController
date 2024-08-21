package org.firstinspires.ftc.teamcode.fy23.processors;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** A suite of tools for controlling acceleration.
 * <p>AccelLimiter (currently?) only works in one dimension. It can do three things:</p>
 * <ul>
 *     <li>Limit acceleration on one or more devices<ul>
 *         <li>When limiting on multiple devices, it maintains the proportions between them so that they behave the
 *         way you'd expect.</li>
 *     </ul></li>
 *     <li>Calculate stopping distance</li>
 *     <li>Ramp up and down to a target position<ul>
 *         <li>This will likely be split out into one or more separate classes in the near future.</li>
 *     </ul></li>
 * </ul>
 *
 * <p>There is one major rule to using AccelLimiter: <b>Be consistent with your units!</b> If you use ticks per second when
 * creating an AccelLimiter object, you must continue using ticks per second in all subsequent uses of it.</p>
 *
 * <hr>
 *
 * <h2>Creating new AccelLimiter objects</h2>
 * <p>Each thing you want to limit acceleration on should have its own instance of AccelLimiter. (This is because each instance
 * only stores and uses one set of parameters.)</p>
 * <p>To create a new AccelLimiter object, you need to determine two things. You can use any unit of acceleration you want, as long as you
 * use the same unit everywhere. You can technically use motor power if encoders are not available, but it is not
 * recommended. More common units are meters per second per second (if you really need real-world relatability) or, ideally for
 * FTC, ticks per second per second (t/s²). With t/s², the motor.getVelocity() and motor.setVelocity() methods will already
 * work in the units we need. Anyway, the two things we need to determine are:</p>
 * <ul>
 *     <li>Maximum Acceleration</li>
 *     <li>Maxinum change in velocity each time a request*() method is called<ul>
 *         <li>With this, if one iteration of the program loop takes longer than normal, the speed won't suddenly increase
 *         more than we expect.</li>
 *     </ul></li>
 * </ul>
 *
 * <p>Pass these parameters into the constructor:</p>
 * <pre><code>@Override
 * public void init() {
 *     double maxAccel = 100; // For our purposes, this is in t/s².
 *     double maxDeltaVEachLoop = 10;
 *     AccelLimiter exampleAL = new AccelLimiter(maxAccel, maxDeltaVEachLoop);
 * // method to be continued...</code></pre>
 *
 * <hr>
 *
 * <h2>Limiting Acceleration on a Single Device</h2>
 * <p>There are two methods that you can call in every iteration of your loop to do this:</p>
 * <ul>
 *     <li><code>public double requestDeltaVel(double deltaVel, double currentTime)</code><ul>
 *         <li>Takes a change in velocity and limits it based on the given maximum acceleration and how much time has
 *         passed since it was called last.</li></ul></li>
 *     <li><code>public double requestVel(double newVel, double currentVel, double currentTime)</code><ul>
 *         <li>Does the same thing, but takes and returns the entire velocity rather than only taking the change. (This
 *         is shorthand. Under the hood, it figures out the change, limits that, then adds it back to the rest.)</li></ul></li>
 * </ul>
 * <p>Let's limit the acceleration on a single motor using the AccelLimiter we created earlier. We'll use a DcMotor named
 * "exampleMotor" and the requestVel() method.</p>
 * <p>Remember that our max. acceleration is 100 t/s². 100 t/s will be our max. drive velocity, so it will take us 1 second
 * to reach full speed.</p>
 * <p>Also, since our maxDeltaVEachLoop is 10 t/s², if a loop takes longer than 100ms, it won't increase its speed more than
 * this limit on that loop.</p>
 * <pre><code>
 *     // We need to pass the time into the request*() methods so AccelLimiter knows how much time passes between calls.
 *     ElapsedTime stopwatch = new ElapsedTime();
 *
 *     DcMotorEx exampleMotor = hardwareMap.get(DcMotorEx.class, "motor");
 * } // end init()
 *
 * {@literal @}Override
 * public void loop() {
 *     // "Requested", as in what the driver wants to do
 *     // Let's say that our drive velocity runs from -100 to +100 ticks per second.
 *     double requestedDriveVelocity = (gamepad1.right_trigger - gamepad1.left_trigger) * 100;
 *
 *     // The user request can jump from 0 to 100.
 *     // AccelLimiter will only change it as fast as maxAccel allows it to.
 *     // Here, if a loop takes 50ms, it will only change it by 5 t/s that loop.
 *     double limitedDriveVelocity = exampleAL.requestVel(requestedDriveVelocity, exampleMotor.getVelocity(), stopwatch.milliseconds());
 *
 *     // If this is a real motor, you should see it gradually ramp up and down as you press the triggers.
 *     exampleMotor.setVelocity(limitedDriveVelocity);
 * }</code></pre>
 * <br>
 * <p>To limit acceleration on multiple motors that act independently (like PixelArm - the pivot and elevator motors do not
 * affect each other), create a separate AccelLimiter object for each. If the motors are connected together, like they
 * are on the drivebase, continue to the next section.</p>
 *
 * <hr>
 *
 * <h2>Limiting Acceleration on Multiple [Interconnected] Devices</h2>
 * <p>This is done with the following method:</p>
 * <br>
 * <code>public List{@literal <}Double{@literal >} requestDeltaVelOnN(List deltaVelList, double currentTime)</code>
 * <br><br>
 * <p>(There is no requestVelOnN() method. You must use change in velocity.)</p>
 * <p>NOTE: This method is meant for multiple motors that are interconnected (the speed of one affects the rest). If your
 * motors act independently (like PixelArm - the pivot and elevator motors do not affect each other), see the previous
 * section.</p>
 * <p>This application is not common enough to include an example here. For an example implementation, look at the code of
 * {@link org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.RRMecanumDriveImpl}.applyDTS().</p>
 *
 * <hr>
 *
 * <h2>Calculating Stopping Distance</h2>
 * <code>public double stoppingDistance(double currentVel, int resolution)</code>
 * <br><br>
 * <p>This method uses the maxAccel parameter already set for its AccelLimiter instance to determine how far the device will
 * continue travelling while it's ramping down from the given velocity.</p>
 * <p>The resolution argument controls how precise the calculation is. A higher resolution yields a more accurate result
 * but takes longer to calculate. 1000 seems to be a happy medium, getting close enough without taking too long.</p>
 * <br>
 * <code>double stoppingDistance = exampleAL.stoppingDistance(motor.getVelocity(), 1000)</code>
 * <br><br>
 * <p>stoppingDistance now (roughly) equals the distance we will continue to travel while we are ramping down, or the
 * distance we need to have available to stop safely.</p>
 * <p>One potential application of this is ramping down towards the end of an actuator's range. The stopping distance will
 * tell you at what point you must start slowing down.</p>
 * <br>
 * <p>Let's consider an example that lets us easily test our example: Our maximum acceleration is 100 t/s², and we're
 * going 100 t/s. The line of our deceleration will cut the square of 100 t/s over 1 second in half, making it a
 * triangle with half the area. In other words, we will tarvel 50 ticks while we're still ramping down, or half the
 * distance we would have traveled at full speed. That is our stopping distance. If we built the example above correctly,
 * stoppingDistance should roughly equal 50.</p>
 * <p>Also, our logic with the graph is the same logic behind a faster way to calculate the stopping distance for linear
 * acceleration:</p>
 * <br>
 * <code>public double simpleStoppingDistance(double currentVel)</code>
 * <br><br>
 * <p>Currently, AccelLimiter only supports linear acceleration, so you may as well use this method. The other methods
 * (which integrate the area under the line of acceleration), however, may enable new functionality in the future (i.e.
 * other acceleration curves).</p>
 *
 * <hr>
 *
 * <h2>Ramping Up/Down to a Target Position</h2>
 * <p>If you're doing this on a single motor, you do not want to use AccelLimiter to do this. Use the SDK's
 * DcMotor.setTargetPosition() method instead.</p>
 * <p>That said, if you have a fun issue like your motor's encoder counting backwards and the SD won't work for you,
 * this might be a stopgap solution until you can fix the motor. What this is really made for, however, is more complex
 * applications such as moving the entire drivetran (4 motors with different encoder positions and rotation directions)
 * until a target position is reached on one of the dead wheels.</p>
 * <p>There is no blocking version of this (yet?). It must be done asynchronously.</p>
 * <br>
 * <code>public void setupRampAlongDistance(double currentPos)</code>
 * <br><br>
 * <p>You can call this at any time to set the target position and the highest speed at which it will travel (perhaps we
 * should call it "cruising speed"). This will reset the AccelLimiter object (!) and put it in the correct internal state
 * for later updateRampAlongDistance() calls to work correctly.</p>
 * <br>
 * <code>public double updateRampALongDistance(double currentPos)</code>
 * <br><br>
 * <p>Call this on every loop <i>after</i> setting up the ramp using the previous method. It will return the velocity
 * that you should send to the motor each loop. See the basic example below:</p>
 * <pre><code>
 * // We want to go to encoder position 1000 at 100 t/s.
 * // We'd put this in our init().
 * exampleAL.setupRampAlongDistance(1000, 100);
 *
 * // Here, we give it our position and it gives us the velocity.
 * // We'd put this in our loop().
 * double applyVel = exampleAL.updateRampAlongDistance(motor.getCurrentPosition());
 *
 * // Here you could do any additional processing on the velocity before sending it to the motor if necessary.
 *
 * motor.setVelocity(applyVel);
 * </code></pre>
 */
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
     * @param reqDeltaV How much you want to change your velocity
     * @param currentTime The current time, in the same unit as the time component of your velocity (ex. if velocity is
     * in meters per second, then currentTime should be in seconds) */
    public double requestDeltaVel(double reqDeltaV, double currentTime) {
        double safeDeltaV = getMaxDeltaVThisLoop(currentTime);
        return Range.clip(reqDeltaV, -safeDeltaV, safeDeltaV);
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
            double maxDeltaVThisLoop = maxAccel * loopTime;
            _lastTime = currentTime;
            return Math.min(maxDeltaVThisLoop, maxDeltaVEachLoop);
        } else {
            _lastTime = currentTime;
            initialized = true;
            return 0;
        }
    }

    /** Request velocity changes on many actuators, and limit them while maintaining the proportions between them. (Works
     * similarly to {@link org.firstinspires.ftc.teamcode.fy23.units.DTS}.normalize().)
     * (Will always return 0 while it initializes the first time it's called since this AccelLimiter was created or
     * reset)
     * This was created for {@link org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive}, but can be used
     * in any similar situation involving multiple interconnected actuators.
     * @param deltaVelList A List{@literal <}Double{@literal >} of all the changes in velocity you are requesting
     * @param currentTime The current time, in the same unit as everything else (ex. if the maximum acceleration is in
     * meters per second, then currentTime should be in seconds) */
    public List<Double> requestDeltaVelOnN(List<Double> deltaVelList, double currentTime) {
        double biggestDeltaV = Math.abs(Collections.max(deltaVelList));
        double maxDeltaV = getMaxDeltaVThisLoop(currentTime);
        // took the absolute value for the condition and scalingFactor calculation later
        if (biggestDeltaV > maxDeltaV) {
            List<Double> returnList = new ArrayList<>();
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
