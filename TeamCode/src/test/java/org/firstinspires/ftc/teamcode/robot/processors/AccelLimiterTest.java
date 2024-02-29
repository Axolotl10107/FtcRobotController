// this class should *not* be used as a sample - please see DTSscalerTest
package org.firstinspires.ftc.teamcode.robot.processors;

import org.firstinspires.ftc.teamcode.fy23.robot.processors.AccelLimiter;
import org.junit.Assert;
import org.junit.Test;

public class AccelLimiterTest {

    @Test
    public void runForwardAtPoints() { // This is *not* a pass/fail test - it just provides debugging information
        // start parameters
        int iterations = 11;
        double requestedVel = 10;
        double lastOutput = 0;
        double currentTime = 0; // need seconds
        double timeStep = 100;

        double maxAccel = 1; // maximum acceleration - it will limit your request to this
        double maxDeltaVEachLoop = 1; // prevent jerks on long loops
        // end parameters
        AccelLimiter accelLimiter = new AccelLimiter(maxAccel, maxDeltaVEachLoop);
        System.out.println(String.format("Requesting {%f} meters per second", requestedVel));
        System.out.println("iter | time | output");
        for (int i = 0; i < iterations; i++) {
            lastOutput = accelLimiter.request(requestedVel, lastOutput, currentTime);
            System.out.println(String.format("{%d} | {%f} | {%f}", i, currentTime, lastOutput));
            currentTime += timeStep;
        }
    }

    @Test
    public void runBackwardAtPoints() { // This is *not* a pass/fail test - it just provides debugging information
        // start parameters
        int iterations = 21;
        double requestedVel = -10;
        double lastOutput = 10; // initial velocity
        double currentTime = 1000;
        double timeStep = 100;

        double maxAccel = 1; // maximum acceleration - it will limit your request to this
        double maxDeltaVEachLoop = 1; // prevent jerks on long loops
        // end parameters
        AccelLimiter accelLimiter = new AccelLimiter(maxAccel, maxDeltaVEachLoop);
        System.out.println(String.format("Requesting {%f} meters per second", requestedVel));
        System.out.println("iter | time | output");
        for (int i = 0; i < iterations; i++) {
            lastOutput = accelLimiter.request(requestedVel, lastOutput, currentTime);
            System.out.println(String.format("{%d} | {%f} | {%f}", i, currentTime, lastOutput));
            currentTime += timeStep;
        }
    }
}
