// this class should *not* be used as a sample - please see DTSscalerTest
package org.firstinspires.ftc.teamcode.fy23.processors;

import org.junit.Assert;
import org.junit.Test;

import java.util.Arrays;
import java.util.List;

public class AccelLimiterTest {

    @Test
    // This is *not* a pass/fail test - it just provides debugging information
    public void runForwardAtPoints() {
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
            lastOutput = accelLimiter.requestVel(requestedVel, lastOutput, currentTime);
            System.out.println(String.format("{%d} | {%f} | {%f}", i, currentTime, lastOutput));
            currentTime += timeStep;
        }
    }

    @Test
    // This is *not* a pass/fail test - it just provides debugging information
    public void runBackwardAtPoints() {
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
            lastOutput = accelLimiter.requestVel(requestedVel, lastOutput, currentTime);
            System.out.println(String.format("{%d} | {%f} | {%f}", i, currentTime, lastOutput));
            currentTime += timeStep;
        }
    }

    @Test
    // This is *not* a pass/fail test.
    public void simplePathPrinter() {
        // start parameters
        double currentTime = 0;
        double lastOutput = 0;
        List requestList = Arrays.asList(0.0, 10.0, 10.0, 20.0, 20.0, 10.0, 10.0, 0.0, 0.0);
        List timeList = Arrays.asList(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        // end parameters
        double maxAccel = 5;
        double maxDeltaVEachLoop = 5;
        AccelLimiter accelLimiter = new AccelLimiter(maxAccel, maxDeltaVEachLoop);
        System.out.println("iter | time | requested | output");
        for (int i=0; i < requestList.size(); i++) {
            currentTime = (double) timeList.get(i);
            lastOutput = accelLimiter.requestVel((double) requestList.get(i), lastOutput, (double) timeList.get(i));
            System.out.println(String.format("{%d} | {%f} | {%f} | {%f}", i, currentTime, (double) requestList.get(i), lastOutput));
        }
    }

    public void plannedPathTest(double maxAccel, double maxDeltaVEachLoop, List<Double> requestList, List<Double> timeList, List<Double> expectedList) {
        // start parameters
        double currentTime = 0;
        double lastOutput = 0;
        int failureCount = 0;
        // end parameters
//        double maxAccel = 5;
//        double maxDeltaVEachLoop = 5;
        AccelLimiter accelLimiter = new AccelLimiter(maxAccel, maxDeltaVEachLoop);
        System.out.println("iter | time | requested | output | expected");
        for (int i=0; i < requestList.size(); i++) {
            lastOutput = accelLimiter.requestVel(requestList.get(i), lastOutput, timeList.get(i));
            System.out.println(String.format("{%d} | {%f} | {%f} | {%f} | {%f}", i, timeList.get(i), requestList.get(i), lastOutput, expectedList.get(i)));
//            Assert.assertEquals(expectedList.get(i), lastOutput, 0.01);
            if (Math.abs(expectedList.get(i) - lastOutput) > 0.01) {
                failureCount += 1;
            }
        }
        if (failureCount > 0) { System.out.println("Failed " + failureCount + " times."); }
        Assert.assertEquals(0, failureCount);
    }

//    @Test
//    public void templateSimplePathTest() {
//        List<Double> requestList = Arrays.asList();
//        List<Double> timeList = Arrays.asList();
//        List<Double> expectedList = Arrays.asList();
//        plannedPathTest(5, 5, requestList, timeList, expectedList);
    // maxAccel and maxDeltaVEachLoop must match what you entered into the list maker
//    }

    @Test
    public void smallSimplePathTest() {
        List<Double> requestList = Arrays.asList(0.0, 10.0, 10.0, 20.0, 20.0, 10.0, 10.0, 0.0, 0.0);
        List<Double> timeList = Arrays.asList(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        List<Double> expectedList = Arrays.asList(0.0, 5.0, 10.0, 15.0, 20.0, 15.0, 10.0, 5.0, 0.0);
        plannedPathTest(5, 5, requestList, timeList, expectedList);
    }

    @Test
    public void largeSimplePathTest() {
        List<Double> requestList = Arrays.asList(0.0, 23.0, 64.0, 64.0, 64.0, 47.0, 25.0, 64.0, 64.0, 64.0, 31.0, 31.0, 0.0, 0.0);
        List<Double> timeList = Arrays.asList(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0);
        List<Double> expectedList = Arrays.asList(0.0, 13.0, 26.0, 39.0, 52.0, 47.0, 34.0, 47.0, 60.0, 64.0, 51.0, 38.0, 25.0, 12.0);
        plannedPathTest(15, 13, requestList, timeList, expectedList);
    }

    @Test
    public void hugeSimplePathTest() {
        List<Double> requestList = Arrays.asList(12.0, 32.0, 32.0, 54.0, 54.0, 45.0, 67.0, 67.0, 47.0, 47.0, 85.0, 85.0, 85.0, 85.0, 34.0, 34.0, 34.0, 34.0, 34.0, 34.0, 57.0, 57.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, 21.0, 21.0, 21.0, 21.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 48.0, 48.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -48.0, -48.0, -48.0, -48.0, -48.0, -48.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        List<Double> timeList = Arrays.asList(0.0, 1.5, 3.0, 4.5, 6.0, 7.5, 9.0, 10.5, 12.0, 13.5, 15.0, 16.5, 18.0, 19.5, 21.0, 22.5, 24.0, 25.5, 27.0, 28.5, 30.0, 31.5, 33.0, 34.5, 36.0, 37.5, 39.0, 40.5, 42.0, 43.5, 45.0, 46.5, 48.0, 49.5, 51.0, 52.5, 54.0, 55.5, 57.0, 58.5, 60.0, 61.5, 63.0, 64.5, 66.0, 67.5, 69.0, 70.5, 72.0, 73.5, 75.0, 76.5, 78.0, 79.5, 81.0, 82.5, 84.0, 85.5, 87.0, 88.5, 90.0, 91.5, 93.0, 94.5, 96.0, 97.5, 99.0);
        List<Double> expectedList = Arrays.asList(0.0, 8.0, 16.0, 24.0, 32.0, 40.0, 48.0, 56.0, 48.0, 47.0, 55.0, 63.0, 71.0, 79.0, 71.0, 63.0, 55.0, 47.0, 39.0, 34.0, 42.0, 50.0, 42.0, 34.0, 26.0, 18.0, 10.0, 2.0, -6.0, -14.0, -15.0, -7.0, 1.0, 9.0, 17.0, 9.0, 1.0, -7.0, -15.0, -23.0, -30.0, -22.0, -14.0, -6.0, 2.0, 10.0, 18.0, 26.0, 34.0, 26.0, 18.0, 10.0, 2.0, 0.0, 0.0, -8.0, -16.0, -24.0, -32.0, -40.0, -48.0, -40.0, -32.0, -24.0, -16.0, -8.0, 0.0);
        plannedPathTest(8, 8, requestList, timeList, expectedList);
    }

    @Test
    public void randomFromHugeSimplePathTest() {
        List<Double> requestList = Arrays.asList(32.0, 67.0, 85.0, 34.0, 57.0, -15.0, -15.0, -15.0, 21.0, -30.0, -30.0, 25.0, 25.0, 48.0, 48.0, 0.0, 0.0, -48.0, -48.0, 0.0, 0.0);
        List<Double> timeList = Arrays.asList(3.0, 9.0, 16.5, 24.0, 31.5, 34.5, 36.0, 43.5, 49.5, 54.0, 57.0, 64.5, 67.5, 70.5, 72.0, 75.0, 81.0, 85.5, 88.5, 91.5, 99.0);
        List<Double> expectedList = Arrays.asList(0.0, 8.0, 16.0, 24.0, 32.0, 24.0, 16.0, 8.0, 16.0, 8.0, 0.0, 8.0, 16.0, 24.0, 32.0, 24.0, 16.0, 8.0, 0.0, 0.0, 0.0);
        plannedPathTest(8, 8, requestList, timeList, expectedList);
    }
}
