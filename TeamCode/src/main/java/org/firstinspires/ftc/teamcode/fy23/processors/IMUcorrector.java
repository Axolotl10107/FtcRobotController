package org.firstinspires.ftc.teamcode.fy23.processors;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** Uses the IMU to actively maintain the current heading unless a deliberate turn is detected. Also lets you "square up".*/
public class IMUcorrector {

    public static class Parameters {
        /** Please pass through the Robot's IMU ( robot.imu ). */
        public FriendlyIMU imu;
        /** Pass in an object, already instantiated and configured */
        public TunablePID pid;
        /** Maximum correction power that can be applied */
        public double maxCorrection = 0.1;
        /** Minimum actionable heading error (in degrees) */
        public double hdgErrTolerance = 1.0;
        /** Minimum absolute value of the turn axis that is considered an intentional turn (which will pause correction) */
        public double turnThreshold = 0.05;
        /** Proximity to the target (in degrees) that counts as hitting the target */
        public double haveHitTargetTolerance = 0.1;
        /** An ElapsedTime (or MockElapsedTime for testing) */
        public ElapsedTime errorSampleTimer = new ElapsedTime();
        /** How long to wait between updates of lastHdgError (milliseconds) (default 1150) */
        public int errorSampleDelay = 1150;
    }

    // __Positive turn is counterclockwise!__ That's just how the IMU works.

    // configuration
    private double maxCorrection;
    private double hdgErrTolerance;
    private double turnThreshold;
    private double haveHitTargetTolerance;

    private double targetHeading = 0; // public for telemetry
    private double headingError = 0; // public for telemetry
    private double lastHdgError = 0; // public for telemetry
    private double lastTurn = 0;
    private double currentTurn = 0;
    private double lastHeading = 0;
    //    public boolean squaringUp = false;
    private boolean haveHitTarget = false;
    private boolean turning = false;

    private DTS returnDTS;

    private FriendlyIMU imu;
    private TunablePID pid;

    private ElapsedTime errorSampleTimer;
    private int errorSampleDelay;
//    public ElapsedTime postSquaringUpPatienceTimer;

    public IMUcorrector(Parameters parameters) {
        maxCorrection = parameters.maxCorrection;
        hdgErrTolerance = parameters.hdgErrTolerance;
        turnThreshold = parameters.turnThreshold;
        haveHitTargetTolerance = parameters.haveHitTargetTolerance;
        imu = parameters.imu;
        pid = parameters.pid;
        errorSampleTimer = parameters.errorSampleTimer;
        errorSampleDelay = parameters.errorSampleDelay;
    }

    private DTS applyCorrection(DTS dts) {
        double correctionPower = pid.getCorrectionPower(headingError, lastHdgError);
        double safeCorrectionPower = Range.clip(correctionPower, -maxCorrection, maxCorrection);
        return dts.withTurn(safeCorrectionPower);
    }

    /** The drive and strafe values will remain unmodified, but the turn power will be replaced by the correction power
     * given by the {@link TunablePID} instance given in the Parameters. */
    public DTS correctDTS(DTS driver) {

//        returnDTS = new DTS(driver.drive, 0, driver.strafe); // we'll populate turn ourselves
        returnDTS = driver; // actually, let's replace turn when we need to instead of never passing theirs through :)

        if (errorSampleTimer.milliseconds() > errorSampleDelay) {
            lastHdgError = headingError;
            errorSampleTimer.reset();
        }

        double currentHeading = imu.yaw();
        headingError = currentHeading - targetHeading;
        // positive (clockwise) error generates positive (counterclockwise) correction

        if (turning) {
            lastTurn = currentTurn;
            currentTurn = currentHeading - lastHeading;
            if (Math.abs(currentTurn) < turnThreshold && Math.abs(lastTurn) > turnThreshold) {
                // if we just fell below the turn threshold
                // Note to self: this is still inside the "if (turning)! This doesn't trigger if the driver didn't initiate the turn with the turn axis.
                turning = false;
                // clear old cumulative values
                pid.clearIntegral();
                pid.clearDerivative();
                lastHdgError = 0;
                targetHeading = currentHeading; // set the target to the direction the driver wants to go now
            }
        } else {
            if (Math.abs(driver.turn) > turnThreshold) {
                // if the driver is requesting to turn
                turning = true;
                haveHitTarget = false;
            } else if (!haveHitTarget || Math.abs(headingError) > hdgErrTolerance) {
                returnDTS = applyCorrection(returnDTS);
                if (Math.abs(headingError) < haveHitTargetTolerance) {
                    // we want to get to about the middle of our heading tolerance, not the edge - mostly for squaring up
                    haveHitTarget = true;
                }
            }
        }

        lastHeading = currentHeading;

        return returnDTS;
    }

    /** Sets the target heading to the nearest cardinal direction. */
    public void squareUp() {
        targetHeading = 90 * Math.round(targetHeading / 90);
        haveHitTarget = false;
    }


    public double getTargetHeading() {
        return targetHeading;
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public double getHeadingError() {
        return headingError;
    }

    public double getLastHeadingError() {
        return lastHdgError;
    }

    public boolean haveHitTarget() {
        return haveHitTarget;
    }

    public boolean isTurning() {
        return turning;
    }
}
