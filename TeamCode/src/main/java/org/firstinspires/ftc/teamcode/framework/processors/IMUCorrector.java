package org.firstinspires.ftc.teamcode.framework.processors;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMU;
import org.firstinspires.ftc.teamcode.framework.units.DTS;

/** Uses the IMU to actively maintain the current heading unless a deliberate turn is detected.
 * Also lets you "square up", or use the IMU to accurately turn to the nearest cardinal direction. */
public class IMUCorrector {

    // A quick warning: __Positive turn is counterclockwise!__ IMU coordinates are right-handed:
    // IRW: This is because of the "right-hand rule." In order for the +Z axis to be up, going from +X to +Y (positive
    // rotation) must be counterclockwise. The axes are probably +X=right, +Y=forward, +Z=up. If positive rotations were
    // clockwise then +Z would be down and +X would probably be forward: +X=forward, +Y=right, +Z=down -OR- +X=left,
    // +Y=forward, +Z=down. The "right-hand rule" is convention and some math you might look up (e.g. 3D rotation)
    // assumes a right-handed coordinate system and may not work correctly (without modification) if using a left-handed
    // system.
    // https://en.wikipedia.org/wiki/Right-hand_rule
    // https://www.evl.uic.edu/ralph/508S98/coordinates.html

    /** Please read if you're using IMUCorrector - important information inside. */
    public static class Parameters {
        /** Create a new IMUCorrector.Parameters object and supply non-optional parameters.
         * @param imu Pass the Robot's IMU ( robot.imu ) through.
         * @param pid Pass in an object, already instantiated and configured. */
        public Parameters(FriendlyIMU imu, TunablePID pid) {
            this.imu = imu;
            this.pid = pid;
        }

        /** You already set this in the constructor and cannot set it again. */
        public final FriendlyIMU imu;

        /** You already set this in the constructor and cannot set it again. */
        public final TunablePID pid;

        /** Maximum correction power that can be applied */
        public double maxCorrectionPower = 0.1;

        /** Minimum absolute value of the turn axis that is considered an intentional turn (which will pause correction) */
        public double turnPowerThreshold = 0.05;

        /** Minimum actionable heading error (in degrees) */
        public double hdgErrToleranceDegrees = 1.0;

        /** A smaller error tolerance for when we're trying to hit the center of the larger tolerance.
         * IMUCorrector used to be content and stop correcting once it was within the hdgErrTolerance. Really, it had
         * only hit the very edge of that tolerance, and it would ride the edge for a while, causing the robot to
         * vibrate. So now, at first, it makes sure to go to the center of the tolerance, and only once it gets there
         * will it expand to the wider tolerance to allow a little drift. But we'll never be at *exactly* 90.000000000
         * degrees, so what counts as the center? That's where this value comes in. Make sure it's much smaller than
         * hdgErrorToleranceDegrees. */
        public double haveHitTargetToleranceDegrees = 0.1;

        /** An ElapsedTime (or MockElapsedTime for testing) */
//        public ElapsedTime errorSampleTimer = new ElapsedTime();
        /** How long to wait between updates of lastHdgError (milliseconds) (default 1150)
         * Changing the default is not recommended, as smaller numbers cause issues. The reason why is unknown. If you
         * find out why this value is necessary, please update this comment with your findings. */
//        public int errorSampleDelay = 1150;
    }

    // configuration
    // IRW: Would it be simpler to store a `private Parameters params` than to copy the fields?
    private double maxCorrectionPower;
    private double hdgErrToleranceDegrees;
    private double turnPowerThreshold;
    private double haveHitTargetToleranceDegrees;

    // state
    private double targetHeading = 0;
    private double headingError = 0;
    private double lastHeadingError = 0;
    private boolean haveHitTarget = false;
    private boolean wasTurning = false;

    private FriendlyIMU imu;
    private TunablePID pid;

//    private ElapsedTime errorSampleTimer;
//    private int errorSampleDelay;

    public IMUCorrector(Parameters parameters) {
        maxCorrectionPower = parameters.maxCorrectionPower;
        hdgErrToleranceDegrees = parameters.hdgErrToleranceDegrees;
        turnPowerThreshold = parameters.turnPowerThreshold;
        haveHitTargetToleranceDegrees = parameters.haveHitTargetToleranceDegrees;
        imu = parameters.imu;
        pid = parameters.pid;
//        errorSampleTimer = parameters.errorSampleTimer;
//        errorSampleDelay = parameters.errorSampleDelay;
    }

    // Make a heading fall within the range [-180, 180).
    // We want our headingError in the same range as the IMU readings, and this particular range ensures that we'll turn
    // in the direction which gets us to the target heading faster.
    private double normalizeHeading(double heading) {
        while (heading > 180) {
            heading -= 360;
        }
        while (heading <= -180) {
            heading += 360;
        }
        return heading;
    }

    /** The drive and strafe values will remain unmodified, but the turn power will be replaced by the correction power
     * given by the {@link TunablePID} instance given in the Parameters.
     * @param driver The DTS that represents what the driver wants to do (i.e. the requested turn power). */
    public DTS correctDTS(DTS driver) {
        final double currentHeading = imu.yaw();
        final DTS returnDTS; // unassigned final value must be set exactly once along every path

        if (Math.abs(driver.turn) > turnPowerThreshold) { // the driver is currently turning
            if (!wasTurning) { // they weren't turning before
                wasTurning = true; // next time, they will have been turning before
            }
            returnDTS = driver; // pass their input through - no correction needed

        } else { // the driver is not currently turning
            if (wasTurning) { // they were turning but have now stopped
                targetHeading = currentHeading; // remember their choice
                haveHitTarget = true; // they turned to exactly where they want to be

                wasTurning = false; // next time, they won't have been turning last time

                // clear old persistent stuff
                pid.clearIntegral();
                lastHeadingError = 0;
            }

            // Positive heading is counterclockwise, but positive correction power turns clockwise.
            // Thus, a positive heading error should generate positive correction power.
            // When we give our PID algorithm the positive error, it will return positive correction (assuming kP is positive).
            // Example: current=240, target=180, difference is +60 (we're more counterclockwise than we want), correction is +60 (go clockwise)
            headingError = normalizeHeading(currentHeading - targetHeading); // see comment on normalizeHeading()

            double errorTolerance;
            if (haveHitTarget) { // we've gotten close enough to our target
                errorTolerance = hdgErrToleranceDegrees; // we'll let some drift slide
            } else { // we could be within the larger tolerance, but we want to get closer to the center first
                errorTolerance = haveHitTargetToleranceDegrees; // we won't allow any (noticeable) drift yet
            }

            if (headingError < errorTolerance) { // we're within our tolerance
                returnDTS = driver.withTurn(0); // no correction needed
            } else { // we're outside of our tolerance
                double correctionPower = pid.correctFor(headingError); // What does the PID algorithm think?
                // How drastic are we willing to be?
                double safeCorrectionPower = Range.clip(correctionPower, -maxCorrectionPower, maxCorrectionPower);
                returnDTS = driver.withTurn(safeCorrectionPower); // Send our correction power to the drivebase.
            }
        }

        // clean up
        lastHeadingError = headingError;

        return returnDTS;
    }



    // For when the comments just make it feel too big and intimidating
    private DTS miniaturizedCorrectDTS(DTS driver) {
        final double currentHeading = imu.yaw();
        final DTS returnDTS;

        if (Math.abs(driver.turn) > turnPowerThreshold) { // currently turning
            if (!wasTurning) {
                wasTurning = true;
            }
            returnDTS = driver;

        } else { // not currently turning
            if (wasTurning) { // clean up from previous turn
                targetHeading = currentHeading;
                haveHitTarget = true;
                wasTurning = false;
                pid.clearIntegral();
                lastHeadingError = 0;
            }

            headingError = currentHeading - targetHeading;

            double errorTolerance; // choose a tolerance
            if (haveHitTarget) {
                errorTolerance = hdgErrToleranceDegrees;
            } else {
                errorTolerance = haveHitTargetToleranceDegrees;
            }

            if (headingError < errorTolerance) { // apply correction if needed
                returnDTS = driver.withTurn(0);
            } else {
                double correctionPower = pid.correctFor(headingError);
                double safeCorrectionPower = Range.clip(correctionPower, -maxCorrectionPower, maxCorrectionPower);
                returnDTS = driver.withTurn(safeCorrectionPower);
            }
        }

        lastHeadingError = headingError; // clean up and return
        return returnDTS;
    }



    /** Sets the target heading to the nearest cardinal direction. */
    public void squareUp() {
        targetHeading = 90 * Math.round(targetHeading / 90);

        // Let correctDTS() know to get closer to the target than the normal tolerance would have it,
        // and then once it's gotten near the center, it will use the larger tolerance.
        haveHitTarget = false;
    }

    /** Returns the target heading (in degrees). */
    public double getTargetHeading() {
        return targetHeading;
    }

    /** Sets the target heading (in degrees). */
    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
        // IRW: Set `haveHitTarget = false`?
    }

    /** Returns how far away the heading is from the target (in degrees). */
    public double getHeadingError() {
        return headingError;
    }

    /** This is probably only useful for debugging IMUCorrector. */
    public double getLastHeadingError() {
        return lastHeadingError;
    }

    // [object] hasHitTarget() in the third person, and [I] haveHitTarget in the first person
    /** Have you gotten to the center of the hdgErrTolerance since last time you turned or squared up?
     * See the description for haveHitTargetToleranceDegrees in the {@link Parameters}. */
    public boolean hasHitTarget() {
        return haveHitTarget;
    }

    /** Returns true if IMUCorrector thinks that the driver is currently trying to turn. */
    public boolean isTurning() {
        return wasTurning; // Maybe it's one loop off, but it's close enough.
    }
}
