package org.firstinspires.ftc.teamcode.fy23.robot.old;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;
import org.firstinspires.ftc.teamcode.fy23.units.PIDConsts;

/** Uses the IMU to actively maintain the current heading unless a deliberate turn is detected.
 * <b>This class has an open task:</b> Filters / Make IMUcorrector Testable */
public class IMUcorrectorBackupTwo {

    // __Positive turn is counterclockwise!__ That's just how the IMU works.

    // configuration
    private double maxTotalCorrection = 0.3;
    private double hdgErrThresholdStill = 1.0;
    private double hdgErrThresholdMoving = 1.0;
    //minimum actionable heading error
    private double turnThreshold = 0.1;
    // how much you must be turning for heading maintenance to temporarily stop
    // and a new target heading to be set when you're done turning
    private double movementThreshold = 0.1;
    // how much you must be moving for the moving threshold, rather than the still threshold, to come into effect
    private int postSquaringUpPatience = 1000; // in milliseconds

    public FriendlyIMUImpl imu; // public for telemetry
    public TunablePID pid; // public for telemetry

    public double targetHeading = 0; // public for telemetry
    public double headingError = 0; // public for telemetry
    public double lastError = 0; // public for telemetry
    private double lastTurn = 0;
    private double lastHeading = 0;
    public boolean squaringUp = false;

    private DTS returnDTS;

    private ElapsedTime errorSampleTimer;
    public ElapsedTime postSquaringUpPatienceTimer;

    public IMUcorrectorBackupTwo(HardwareMap hardwareMap, double p, double im, double maxI, double dm) {
        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu = new FriendlyIMUImpl(imuParams, hardwareMap);
        pid = new TunablePID(p, im, maxI, dm);
        errorSampleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        postSquaringUpPatienceTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public IMUcorrectorBackupTwo(HardwareMap hardwareMap, PIDConsts pidConsts) { // function overloading
        this(hardwareMap, pidConsts.kP, pidConsts.kI, pidConsts.maxI, pidConsts.kD);
    }

    private DTS applyCorrection(DTS dts) {
        double correctionPower = pid.correctFor(headingError);
        double safeCorrectionPower = Range.clip(correctionPower, -maxTotalCorrection, maxTotalCorrection);
        return dts.withTurn(safeCorrectionPower);
    }

    private boolean moving(DTS dts) {
        return (Math.abs(dts.drive) + Math.abs(dts.strafe) > movementThreshold);
    }

    /** The drive and strafe values will remain unmodified, but it will <b>add</b> correction to the turn value. */
    public DTS correctDTS(DTS dts) {

        returnDTS = new DTS(dts.drive, 0, dts.strafe); // we'll populate turn ourselves

        if (errorSampleTimer.milliseconds() > 1150) {
            lastError = headingError;
            errorSampleTimer.reset();
        }
        headingError = targetHeading - imu.yaw(); //imu.yaw() returns our heading

        if (Math.abs(dts.turn) > turnThreshold) {
            returnDTS = dts; // if the driver is turning, let them turn
        } else {
            if ((Math.abs(lastTurn) > turnThreshold && Math.abs(imu.yaw() - lastHeading) < turnThreshold)
                    && (!squaringUp && postSquaringUpPatienceTimer.milliseconds() > postSquaringUpPatience)) {
                // if we're actually done turning (we just fell below the turning threshold
                targetHeading = imu.yaw(); // we want to face the direction they turned to now
                pid.clearIntegral(); // accumulated corrections are irrelevant now
                headingError = 0; // this will go into lastError
            }
            if (moving(dts) || squaringUp) {
                if (Math.abs(headingError) > hdgErrThresholdMoving) {
                    // if we're above the threshold
                    returnDTS = applyCorrection(dts);
                }
            } else if (Math.abs(headingError) > hdgErrThresholdStill) {
                // if we're not moving and above the still threshold
                returnDTS = applyCorrection(dts);
            } else {
                returnDTS = dts; // we're not moving and we're within tolerances, so don't bother with correction
            }

            if (squaringUp) {
                if (moving(dts)) {
                    if (Math.abs(headingError) <= hdgErrThresholdMoving) {
                        squaringUp = false;
                        postSquaringUpPatienceTimer.reset();
                    }
                } else {
                    if (Math.abs(headingError) <= hdgErrThresholdStill) {
                        squaringUp = false;
                        postSquaringUpPatienceTimer.reset();
                    }
                }
            }
        }

        lastTurn = Math.abs(imu.yaw() - lastHeading);
        lastHeading = imu.yaw();

        return returnDTS;
    }

    /** Set the target heading to the nearest cardinal direction */
    public void squareUp() {
        targetHeading = 90 * Math.round(targetHeading / 90);
        squaringUp = true;
    }
}
