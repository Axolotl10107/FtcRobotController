// this class can function as a sample of a unit test
package org.firstinspires.ftc.teamcode.fy23.robot.old;

import org.junit.Assert;

import org.firstinspires.ftc.teamcode.fy23.units.DTS;
import org.junit.Test;

public class DTSscalerTest { // click the play button on the left to run all tests in this class

    DTSscaler dtsScaler = new DTSscaler();

    @Test // tells JUnit that the method below this is a test, which gives it a play button
    public void testSum() {
        int iterations = 10;
        for (int i = 0; i < iterations; i++) { // click the play button on the left to run just this test
            DTS randomDTS = new DTS(Math.random(), Math.random(), Math.random());
            DTS scaledDTS = dtsScaler.scale(randomDTS);
            double sum = scaledDTS.drive + scaledDTS.turn + scaledDTS.strafe;
            Assert.assertTrue(sum <= 1); // here we make sure that stuff is correct
            // if it isn't, the test fails
            // in this case, if the sum of the values in our scaled DTS is greater than 1, the test fails
        }
    }

    @Test
    public void testRatios() {
        int iterations = 10;
        for (int i = 0; i < iterations; i++) {
            DTS randomDTS = new DTS(Math.random(), Math.random(), Math.random());
            DTS scaledDTS = dtsScaler.scale(randomDTS);
            Assert.assertEquals((randomDTS.drive / randomDTS.turn), (scaledDTS.drive / scaledDTS.turn), 0.01);
            // extra value at the end is tolerance - floating point numbers tend not to be *exactly* equal
            Assert.assertEquals((randomDTS.turn / randomDTS.strafe), (scaledDTS.turn / scaledDTS.strafe), 0.01);
            Assert.assertEquals((randomDTS.drive / randomDTS.strafe), (scaledDTS.drive / scaledDTS.strafe), 0.01);
            // if the ratios between values in the scaled dts are different from the ones we gave DTSscaler, the test fails
        }
    }

}
