package org.firstinspires.ftc.teamcode.framework.processors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fakestuff.MockElapsedTime;
import org.junit.Assert;
import org.junit.Test;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

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
    public void smallPlannedPathTest() {
        List<Double> requestList = Arrays.asList(0.0, 10.0, 10.0, 20.0, 20.0, 10.0, 10.0, 0.0, 0.0);
        List<Double> timeList = Arrays.asList(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        List<Double> expectedList = Arrays.asList(0.0, 5.0, 10.0, 15.0, 20.0, 15.0, 10.0, 5.0, 0.0);
        plannedPathTest(5, 5, requestList, timeList, expectedList);
    }

    @Test
    public void largePlannedPathTest() {
        List<Double> requestList = Arrays.asList(0.0, 23.0, 64.0, 64.0, 64.0, 47.0, 25.0, 64.0, 64.0, 64.0, 31.0, 31.0, 0.0, 0.0);
        List<Double> timeList = Arrays.asList(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0);
        List<Double> expectedList = Arrays.asList(0.0, 13.0, 26.0, 39.0, 52.0, 47.0, 34.0, 47.0, 60.0, 64.0, 51.0, 38.0, 25.0, 12.0);
        plannedPathTest(15, 13, requestList, timeList, expectedList);
    }

    @Test
    public void hugePlannedPathTest() {
        List<Double> requestList = Arrays.asList(12.0, 32.0, 32.0, 54.0, 54.0, 45.0, 67.0, 67.0, 47.0, 47.0, 85.0, 85.0, 85.0, 85.0, 34.0, 34.0, 34.0, 34.0, 34.0, 34.0, 57.0, 57.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, 21.0, 21.0, 21.0, 21.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 48.0, 48.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -48.0, -48.0, -48.0, -48.0, -48.0, -48.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        List<Double> timeList = Arrays.asList(0.0, 1.5, 3.0, 4.5, 6.0, 7.5, 9.0, 10.5, 12.0, 13.5, 15.0, 16.5, 18.0, 19.5, 21.0, 22.5, 24.0, 25.5, 27.0, 28.5, 30.0, 31.5, 33.0, 34.5, 36.0, 37.5, 39.0, 40.5, 42.0, 43.5, 45.0, 46.5, 48.0, 49.5, 51.0, 52.5, 54.0, 55.5, 57.0, 58.5, 60.0, 61.5, 63.0, 64.5, 66.0, 67.5, 69.0, 70.5, 72.0, 73.5, 75.0, 76.5, 78.0, 79.5, 81.0, 82.5, 84.0, 85.5, 87.0, 88.5, 90.0, 91.5, 93.0, 94.5, 96.0, 97.5, 99.0);
        List<Double> expectedList = Arrays.asList(0.0, 8.0, 16.0, 24.0, 32.0, 40.0, 48.0, 56.0, 48.0, 47.0, 55.0, 63.0, 71.0, 79.0, 71.0, 63.0, 55.0, 47.0, 39.0, 34.0, 42.0, 50.0, 42.0, 34.0, 26.0, 18.0, 10.0, 2.0, -6.0, -14.0, -15.0, -7.0, 1.0, 9.0, 17.0, 9.0, 1.0, -7.0, -15.0, -23.0, -30.0, -22.0, -14.0, -6.0, 2.0, 10.0, 18.0, 26.0, 34.0, 26.0, 18.0, 10.0, 2.0, 0.0, 0.0, -8.0, -16.0, -24.0, -32.0, -40.0, -48.0, -40.0, -32.0, -24.0, -16.0, -8.0, 0.0);
        plannedPathTest(8, 8, requestList, timeList, expectedList);
    }

    @Test
    public void randomFromHugePlannedPathTest() {
        List<Double> requestList = Arrays.asList(32.0, 67.0, 85.0, 34.0, 57.0, -15.0, -15.0, -15.0, 21.0, -30.0, -30.0, 25.0, 25.0, 48.0, 48.0, 0.0, 0.0, -48.0, -48.0, 0.0, 0.0);
        List<Double> timeList = Arrays.asList(3.0, 9.0, 16.5, 24.0, 31.5, 34.5, 36.0, 43.5, 49.5, 54.0, 57.0, 64.5, 67.5, 70.5, 72.0, 75.0, 81.0, 85.5, 88.5, 91.5, 99.0);
        List<Double> expectedList = Arrays.asList(0.0, 8.0, 16.0, 24.0, 32.0, 24.0, 16.0, 8.0, 16.0, 8.0, 0.0, 8.0, 16.0, 24.0, 32.0, 24.0, 16.0, 8.0, 0.0, 0.0, 0.0);
        plannedPathTest(8, 8, requestList, timeList, expectedList);
    }

    @Test
    public void requestDeltaVelOnNPrinter() {
        AccelLimiter accelLimiter = new AccelLimiter(1, 10);
        List<Double> deltaVelList = Arrays.asList(10.0, 10.0, 10.0, 10.0);
        double currentTime = 0;
        System.out.println("iter | time | deltaTime | first requested item | first output item");
        for (int i = 0; i < 15; i++) {
            List<Double> outputList = accelLimiter.requestDeltaVelOnN(deltaVelList, currentTime);
            System.out.println(String.format("{%d} | {%f} | {%f} | {%f} | {%f}", i, currentTime, (double) i/10, deltaVelList.get(1), outputList.get(1)));
            currentTime += ((double) (i+1) / 10);
        }
    }

    @Test
    public void requestDeltaVelOnNMaxAccelPassFail1() {
        AccelLimiter accelLimiter = new AccelLimiter(5, 10);
        List<Double> deltaVelList = Arrays.asList(5.0, 5.0, 5.0, 5.0);
        double currentTime = 0;
        List<Double> outputList = accelLimiter.requestDeltaVelOnN(deltaVelList, currentTime);
        // make sure it takes the first loop to initialize
        Assert.assertEquals(Arrays.asList(0.0, 0.0, 0.0, 0.0), outputList);

        currentTime = 0.5;
        outputList = accelLimiter.requestDeltaVelOnN(deltaVelList, currentTime);
        // Now it should give 2.5, since it's been .5 seconds and we can do 5 m/s every second
        Assert.assertEquals(Arrays.asList(2.5, 2.5, 2.5, 2.5), outputList);

        currentTime = 1.5;
        outputList = accelLimiter.requestDeltaVelOnN(deltaVelList, currentTime);
        Assert.assertEquals(Arrays.asList(5.0, 5.0, 5.0, 5.0), outputList);

        deltaVelList = Arrays.asList(10.0, 10.0, 10.0, 10.0);
        currentTime = 2.5;
        outputList = accelLimiter.requestDeltaVelOnN(deltaVelList, currentTime);
        // We requested 10, but that's more than the maxAccel we set allows
        Assert.assertEquals(Arrays.asList(5.0, 5.0, 5.0, 5.0), outputList);
    }

    @Test
    public void requestDeltaVelOnNMaxAccelPassFail2() {
        AccelLimiter accelLimiter = new AccelLimiter(5, 10);
        List<Double> deltaVelList = Arrays.asList(-5.0, -5.0, -5.0, -5.0);
        double currentTime = 0;
        List<Double> outputList = accelLimiter.requestDeltaVelOnN(deltaVelList, currentTime);
        // make sure it takes the first loop to initialize
        Assert.assertEquals(Arrays.asList(-0.0, -0.0, -0.0, -0.0), outputList);

        currentTime = 0.5;
        outputList = accelLimiter.requestDeltaVelOnN(deltaVelList, currentTime);
        // Now it should give -2.5, since it's been .5 seconds and we can do 5 m/s every second
        Assert.assertEquals(Arrays.asList(-2.5, -2.5, -2.5, -2.5), outputList);

        currentTime = 1.5;
        outputList = accelLimiter.requestDeltaVelOnN(deltaVelList, currentTime);
        Assert.assertEquals(Arrays.asList(-5.0, -5.0, -5.0, -5.0), outputList);

        deltaVelList = Arrays.asList(-10.0, -10.0, -10.0, -10.0);
        currentTime = 2.5;
        outputList = accelLimiter.requestDeltaVelOnN(deltaVelList, currentTime);
        // We requested -10, but that's more than the maxAccel we set allows
        Assert.assertEquals(Arrays.asList(-5.0, -5.0, -5.0, -5.0), outputList);
    }

    @Test
    public void manyPixelArmProblemTests() {
        for (int i=0; i<1000; i++) {
            PixelArmProblemTest();
        }
    }

    @Test
    public void PixelArmProblemTest() {
        AccelLimiter accelLimiter = new AccelLimiter(1.0, 0.1);
        MockElapsedTime stopwatch = new MockElapsedTime();
        double output;
        stopwatch.setNanos(TimeUnit.MILLISECONDS.toNanos(1150));

        for (int i=0; i<9; i++) {
            output = accelLimiter.requestVel(0, 0, stopwatch.seconds());
            Assert.assertEquals(0, output, 0.01);
            stopwatch.setNanos((long) (stopwatch.nanoseconds() + (Math.random() * 1000000)));
        }

        output = accelLimiter.requestVel(0.98573, 0, stopwatch.seconds());
        stopwatch.setNanos((long) (stopwatch.nanoseconds() + (Math.random() * 1000000)));
        Assert.assertTrue(output < 0.1); // did not exceed maxDeltaVEachLoop

        output = accelLimiter.requestVel(0.54862, output, stopwatch.seconds());
        stopwatch.setNanos((long) (stopwatch.nanoseconds() + (Math.random() * 1000000)));
        Assert.assertTrue(output < 0.2);


        stopwatch.setNanos(stopwatch.nanoseconds() + TimeUnit.SECONDS.toNanos(1)); // add 1 second
        output = accelLimiter.requestVel(0, output, stopwatch.seconds());
        Assert.assertEquals(0, output, 0.001);
        stopwatch.setNanos(stopwatch.nanoseconds() + TimeUnit.SECONDS.toNanos(1)); // add 1 second


        Assert.assertEquals(0, accelLimiter.requestVel(0, output, stopwatch.seconds()), 0.001);
        output = accelLimiter.requestVel(0.96438, 0, stopwatch.seconds());
        stopwatch.setNanos((long) (stopwatch.nanoseconds() + (Math.random() * 1000000)));
        Assert.assertTrue(output < 0.1); // did not exceed maxDeltaVEachLoop

        output = accelLimiter.requestVel(0.78421, output, stopwatch.seconds());
        stopwatch.setNanos((long) (stopwatch.nanoseconds() + (Math.random() * 1000000)));
        Assert.assertTrue(output < 0.2);


        for (int i=0; i<9; i++) {
            double lastOutput = output;
            stopwatch.setNanos((long) (stopwatch.nanoseconds() + (Math.random() * 1000000)));
            output = accelLimiter.requestVel(0, lastOutput, stopwatch.seconds());
            System.out.println(String.format("lastOutput: {%f}", lastOutput));
            System.out.println(String.format("output: {%f}", output));
            Assert.assertTrue((output - lastOutput) < 0.1);
        }
    }


    @Test
    // This is *not* a pass/fail test.
    public void stoppingDistancePrinter() {
        AccelLimiter accelLimiter = new AccelLimiter(2499, 249.9);
        ElapsedTime stopwatch = new ElapsedTime();
        double stoppingDistance = accelLimiter.stoppingDistance(2499, 1000);
        System.out.println(String.format("Got {%f} meters in {%f} milliseconds.", stoppingDistance, stopwatch.milliseconds()));
    }

    public double stoppingDistanceSimplePassFail(double maxAccel, double maxDeltaVEachLoop, double currentVel, int resolution) {
        AccelLimiter accelLimiter = new AccelLimiter(maxAccel, maxDeltaVEachLoop);
        double stoppingDistance = accelLimiter.stoppingDistance(currentVel, resolution);
        return stoppingDistance;
    }

    @Test
    public void stoppingDistanceSimplePassFail1() {
        Assert.assertEquals(1.25, stoppingDistanceSimplePassFail(10, 1, 5, 1000), 0.01);
    }

    @Test
    public void stoppingDistanceSimplePassFail2() {
        Assert.assertEquals(5, stoppingDistanceSimplePassFail(10, 1, 10, 1000), 0.01);
    }

    @Test
    public void stoppingDistanceSimplePassFail3() {
        Assert.assertEquals(5, stoppingDistanceSimplePassFail(10, 1, -10, 1000), 0.01);
    }

    @Test
    public void stoppingDistanceSimplePassFail4() {
        Assert.assertEquals(45, stoppingDistanceSimplePassFail(10, 10, 30, 1000), 0.02);
    }



    @Test
    public void rampAlongDistancePrinter() {
        // parameters
        double maxAccel = 10; // in meters per second
        double maxDeltaVEachLoop = 10;
        double timeStep = 0.1; // in seconds
        double targetPos = 1000; // in meters
        double maxVelocity = 30;
        AccelLimiter accelLimiter = new AccelLimiter(maxAccel, maxDeltaVEachLoop);
        MockElapsedTime stopwatch = new MockElapsedTime();
        accelLimiter.setupRampToTarget(targetPos, maxVelocity, stopwatch);
        double currentPos = 0;
        double lastOutput;
        int iters = 0;
        System.out.println("iter | time | velocity | position");
        while ((currentPos < (targetPos - 1)) && (iters < 1000)) {
            iters += 1;
            lastOutput = accelLimiter.updateRampToTarget(currentPos);
            currentPos += lastOutput * timeStep;
            System.out.println(String.format("{%d} | {%f} | {%f} | {%f}", iters, stopwatch.seconds(), lastOutput, currentPos));
            stopwatch.setNanos((long) (stopwatch.nanoseconds() + (timeStep * 1000000000)));
        }
    }

//    @Test
    // Broken
    public void rampAlongDistancePathTest() {
        AccelLimiter accelLimiter = new AccelLimiter(5, 5);
        MockElapsedTime stopwatch = new MockElapsedTime();
        accelLimiter.setupRampToTarget(50, 15, stopwatch);
        List<Double> distanceList = Arrays.asList(0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0);
        List<Long> timeList = Arrays.asList(0L, 5L, 10L, 15L, 20L, 25L, 30L, 35L, 40L, 45L, 50L);
        List<Double> expectedVelList = Arrays.asList(0.0, 5.0, 10.0, 15.0, 15.0, 15.0, 15.0, 15.0, 10.0, 5.0, 0.0);
        double lastOutput = 0;
        int failureCount = 0;
        System.out.println("iter | time | distance | output vel | expected vel");
        for (int i=0; i < distanceList.size(); i++) {
            stopwatch.setNanos(TimeUnit.SECONDS.toNanos(timeList.get(i)));
            lastOutput = accelLimiter.updateRampToTarget(distanceList.get(i));
            System.out.println(String.format("{%d} | {%f} | {%f} | {%f} | {%f}", i, stopwatch.seconds(), distanceList.get(i), lastOutput, expectedVelList.get(i)));
            if (Math.abs(expectedVelList.get(i) - lastOutput) > 0.01) {
                failureCount += 1;
            }
        }
        if (failureCount > 0) { System.out.println("Failed " + failureCount + " times."); }
        Assert.assertEquals(0, failureCount);
    }
}
