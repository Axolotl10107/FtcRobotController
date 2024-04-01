package org.firstinspires.ftc.teamcode.fy23.units;

import org.firstinspires.ftc.teamcode.fy23.units.DTS;
import org.junit.Assert;
import org.junit.Test;

public class DTSTest {

    /** if true, random DTS objects will be used over multiple
     * iterations - otherwise, the fixedDTS will be used and each test will be run once */
    boolean randomize = true;

    double equalsDelta = 0.01;

    DTS fixedDTS = new DTS(0.5, 0.3, 0.7);
    DTS modifierDTS = new DTS(0.2, 0.5, 0.4);
    // for with*() tests, make sure fixedDTS.drive != modifierDTS.drive, and same for turn and strafe
    double scaleFactor = 1.2;

    public DTS randomDTS() {
        return new DTS(Math.random(), Math.random(), Math.random());
    }

    private int getIters(int requested) {
        if (randomize) {
             return requested;
        } else {
            return 1;
        }
    }

    private DTS getFirstTestDTS() {
        if (randomize) {
            return randomDTS();
        } else {
            return fixedDTS;
        }
    }


    @Test
    public void testNegate() {
        int iterations = getIters(10);
        DTS testDTS = getFirstTestDTS();
        for (int i = 0; i < iterations; i++) {
            DTS negatedDTS = testDTS.negate();
            Assert.assertEquals(-testDTS.drive, negatedDTS.drive, equalsDelta);
            Assert.assertEquals(-testDTS.turn, negatedDTS.turn, equalsDelta);
            Assert.assertEquals(-testDTS.strafe, negatedDTS.strafe, equalsDelta);
        }
    }

    @Test
    public void testAdd() {
        int iterations = getIters(10);
        DTS testDTS = getFirstTestDTS();
        for (int i = 0; i < iterations; i++) {
            DTS secondDTS = randomize ? randomDTS() : modifierDTS; // in-line if/else statement
            DTS summedDTS = testDTS.add(secondDTS);
            Assert.assertEquals((testDTS.drive + secondDTS.drive), summedDTS.drive, equalsDelta);
            Assert.assertEquals((testDTS.turn + secondDTS.turn), summedDTS.turn, equalsDelta);
            Assert.assertEquals((testDTS.strafe + secondDTS.strafe), summedDTS.strafe, equalsDelta);
        }
    }

    @Test
    public void testScale() {
        int iterations = getIters(10);
        DTS testDTS = getFirstTestDTS();
        for (int i = 0; i < iterations; i++) {
            double factor = randomize ? Math.random() : scaleFactor; // in-line if/else statement
            DTS scaledDTS = testDTS.scale(factor);
            Assert.assertEquals((testDTS.drive * factor), scaledDTS.drive, equalsDelta);
            Assert.assertEquals((testDTS.turn * factor), scaledDTS.turn, equalsDelta);
            Assert.assertEquals((testDTS.strafe * factor), scaledDTS.strafe, equalsDelta);
        }
    }

    @Test
    public void testNormalizeSum() {
        int iterations = getIters(10);
        DTS testDTS = getFirstTestDTS();
        for (int i = 0; i < iterations; i++) {
            DTS normalizedDTS = testDTS.normalize();
            double sum = normalizedDTS.drive + normalizedDTS.turn + normalizedDTS.strafe;
            Assert.assertEquals(1, Math.max(sum, 1), equalsDelta);
            testDTS = randomDTS(); // get a new random DTS
            // at the end, so will do nothing if not in random mode (1 iteration)
        }
    }

    @Test
    public void testNormalizeRatios() {
        int iterations = getIters(10);
        DTS testDTS = getFirstTestDTS();
        for (int i = 0; i < iterations; i++) {
            DTS normalizedDTS = testDTS.normalize();
            Assert.assertEquals((testDTS.drive / testDTS.turn), (normalizedDTS.drive / normalizedDTS.turn), equalsDelta);
            Assert.assertEquals((testDTS.turn / testDTS.strafe), (normalizedDTS.turn / normalizedDTS.strafe), equalsDelta);
            Assert.assertEquals((testDTS.drive / testDTS.strafe), (normalizedDTS.drive / normalizedDTS.strafe), equalsDelta);
        }
    }

    @Test
    public void testWithDrive() {
        DTS resultDTS = fixedDTS.withDrive(modifierDTS.drive);
        Assert.assertEquals(modifierDTS.drive, resultDTS.drive, equalsDelta);
    }

    @Test
    public void testWithTurn() {
        DTS resultDTS = fixedDTS.withTurn(modifierDTS.turn);
        Assert.assertEquals(modifierDTS.turn, resultDTS.turn, equalsDelta);
    }

    @Test
    public void testWithStrafe() {
        DTS resultDTS = fixedDTS.withStrafe(modifierDTS.strafe);
        Assert.assertEquals(modifierDTS.strafe, resultDTS.strafe, equalsDelta);
    }

}
