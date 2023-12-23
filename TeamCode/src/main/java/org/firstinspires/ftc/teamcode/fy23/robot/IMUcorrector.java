package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;
import org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts;

public class IMUcorrector {

    // __Positive turn is counterclockwise!__ That's just how the IMU works.

    // configuration
    private double maxTotalCorrection = 0.3;
    private double hdgErrThreshold = 0.2;
    //minimum actionable heading error
    private double turnThreshold = 0.05;
    // how much you must be turning for heading maintenance to temporarily stop
    // and a new target heading to be set when you're done turning

    public double correctedTurnPower; // for telemetry

    private FriendlyIMU imu;
    private TunablePID pid;

    private double targetHeading = 0;
    private double headingError;
    private double lastError = 0;
    private double lastTurn = 0;

    private DTS returnDTS;

    public IMUcorrector(HardwareMap hardwareMap, double p, double im, double dm) {
        imu = new FriendlyIMU(hardwareMap);
        pid = new TunablePID(p, im, dm);
    }

    public IMUcorrector(HardwareMap hardwareMap, PIDconsts pidConsts) { // function overloading
        imu = new FriendlyIMU(hardwareMap);
        pid = new TunablePID(pidConsts);
    }

    public DTS correctDTS(DTS dts) {

        returnDTS = new DTS(dts.drive, 0, dts.strafe); // we'll populate turn ourselves

        lastError = headingError;
        headingError = targetHeading - imu.yaw(); //imu.yaw() returns our heading

        // just one if statement to actually do the correction
        if (Math.abs(dts.turn) > turnThreshold) {
            returnDTS.turn = dts.turn;
            // if the driver is turning, let them turn
        } else if (Math.abs(headingError) > hdgErrThreshold) {
            returnDTS.turn = Range.clip(pid.getCorrectionPower(headingError, lastError), -maxTotalCorrection, maxTotalCorrection);
            // otherwise, we have it to ourselves :) The TunablePID does the PID for us. We just
            //determine when we can use it and give it the numbers it needs.
        }

        // this if statement is for ourselves
        if (Math.abs(dts.turn) < turnThreshold && lastTurn > turnThreshold) {
            targetHeading = imu.yaw();
            // if they just got done turning, set the current heading as our new target to hold
        }
        lastTurn = Math.abs(dts.turn);

        correctedTurnPower = returnDTS.turn; // for telemetry
        return returnDTS;
    }
}