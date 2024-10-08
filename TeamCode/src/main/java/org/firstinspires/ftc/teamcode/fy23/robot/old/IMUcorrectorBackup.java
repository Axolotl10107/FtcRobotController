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
public class IMUcorrectorBackup {

    // __Positive turn is counterclockwise!__ That's just how the IMU works.

    // configuration
    private double maxTotalCorrection = 0.3;
    private double hdgErrThresholdStill = 0.2;
    private double hdgErrThresholdMoving = 0.2;
    //minimum actionable heading error
    private double turnThreshold = 0.01;
    // how much you must be turning for heading maintenance to temporarily stop
    // and a new target heading to be set when you're done turning

    public double correctedTurnPower; // for telemetry

    public FriendlyIMUImpl imu; // public for telemetry
    public TunablePID pid; // public for telemetry

    public double targetHeading = 0; // public for telemetry
    public double headingError = 0; // public for telemetry
    public double lastError = 0; // public for telemetry
    private double lastTurn = 0;

    private DTS returnDTS;

    private ElapsedTime errorSampleTimer;
    private ElapsedTime pidEnableTimer;

    public IMUcorrectorBackup(HardwareMap hardwareMap, double p, double im, double maxi, double dm) {
        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu = new FriendlyIMUImpl(imuParams, hardwareMap);
        pid = new TunablePID(p, im, maxi, dm);
        errorSampleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        pidEnableTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public IMUcorrectorBackup(HardwareMap hardwareMap, PIDConsts pidConsts) { // function overloading
        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu = new FriendlyIMUImpl(imuParams, hardwareMap);
        pid = new TunablePID(pidConsts.kP, pidConsts.kI, pidConsts.maxI, pidConsts.kD);
        errorSampleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        pidEnableTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /** The drive and strafe values will remain unmodified, but it will <b>add</b> correction to the turn value. */
    public DTS correctDTS(DTS dts) {

        returnDTS = new DTS(dts.drive, 0, dts.strafe); // we'll populate turn ourselves

        if (errorSampleTimer.milliseconds() > 1150) {
            lastError = headingError;
            errorSampleTimer.reset();
        }
        headingError = targetHeading - imu.yaw(); //imu.yaw() returns our heading

        // just one if statement to actually do the correction
        if (Math.abs(dts.turn) > turnThreshold) {
            returnDTS = returnDTS.withTurn(dts.turn);
            // if the driver is turning, let them turn
            pid.clearIntegral();
            pidEnableTimer.reset();
            //we don't need PID while turning
        } else if ((Math.abs(headingError) > hdgErrThresholdStill && Math.abs(dts.drive) < turnThreshold && Math.abs(dts.strafe) < turnThreshold) || (Math.abs(headingError) > hdgErrThresholdMoving && (Math.abs(dts.drive) > turnThreshold) || Math.abs(dts.strafe) > turnThreshold)) {
            returnDTS = returnDTS.withTurn(Range.clip(pid.correctFor(headingError), -maxTotalCorrection, maxTotalCorrection));
            if (pidEnableTimer.milliseconds() < 800) {
                pid.clearIntegral();
            }
            // otherwise, we have it to ourselves :) The TunablePID does the PID for us. We just
            //determine when we can use it and give it the numbers it needs.
        }

        // this if statement is for ourselves
        if (Math.abs(dts.turn) < turnThreshold && lastTurn > turnThreshold) {
            targetHeading = imu.yaw();
            // if they just got done turning, set the current heading as our new target to hold
            pid.clearIntegral();
            headingError = 0; // this will end up in lastError, where I want it, next iteration
            // and reset the accumulated corrections now that they are irrelevant
        }
        lastTurn = Math.abs(dts.turn);

        correctedTurnPower = returnDTS.turn; // for telemetry
        return returnDTS;
    }

    /** Set the target heading to the nearest cardinal direction */
    public void squareUp() {
        targetHeading = 90 * Math.round(targetHeading / 90);
    }
}