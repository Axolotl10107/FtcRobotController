package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.robot.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

/** Represents a mecanum drive base, such as the goBILDA strafer. Make sure to normalize a DTS
 * before passing it in! */
public class MecanumDrive {

    public static class Parameters {
        public boolean present; /** Is the subsystem present on this robot? */

        /** max. individual motor acceleration, in power per second
         * (power loosely represents velocity) */
        public double maxMotorAccel;

        /** prevents jerking - in power */
        public double maxDeltaVEachLoop;

        /** The name of the motor on the left front corner of the drivebase */
        public String leftFrontName;

        /** Direction motor spins when positive power is applied - to drive the motor "backwards",
         * do not set this to reverse! Set the power to a negative value. */
        public DcMotor.Direction leftFrontDirection;

        public String rightFrontName;
        public DcMotor.Direction rightFrontDirection;

        public String leftBackName;
        public DcMotor.Direction leftBackDirection;

        public String rightBackName;
        public DcMotor.Direction rightBackDirection;

        public DcMotor.RunMode runMode; /** Applies to all motors */
        public DcMotor.ZeroPowerBehavior zeroPowerBehavior; /** Applies to all motors */
    }

    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;

    public AccelLimiter accelLimiter;
    public ElapsedTime stopwatch;

    public MecanumDrive(Parameters parameters, HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, parameters.leftFrontName);
        rightFront = hardwareMap.get(DcMotorEx.class, parameters.rightFrontName);
        leftBack = hardwareMap.get(DcMotorEx.class, parameters.leftBackName);
        rightBack = hardwareMap.get(DcMotorEx.class, parameters.rightBackName);

        leftFront.setDirection(parameters.leftFrontDirection);
        rightFront.setDirection(parameters.rightFrontDirection);
        leftBack.setDirection(parameters.leftBackDirection);
        rightBack.setDirection(parameters.rightBackDirection);

        try {
            setMode(parameters.runMode);
        } catch (Exception x) {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // if the runmode wasn't set, pick a default
        }

        try {
            setZeroPowerBehavior(parameters.zeroPowerBehavior);
        } catch (Exception x) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        accelLimiter = new AccelLimiter(parameters.maxMotorAccel, parameters.maxDeltaVEachLoop);
        stopwatch = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /** Takes these components as motor powers */
    public void applyDTS(double drive, double turn, double strafe) {
        leftFront.setPower(drive - turn + strafe);
        rightFront.setPower(drive + turn - strafe);
        leftBack.setPower(drive - turn - strafe);
        rightBack.setPower(drive + turn + strafe);
    }

    /** Takes a DTS of motor powers */
    public void applyDTS(DTS dts) { // function overloading
        applyDTS(dts.drive, dts.turn, dts.strafe);
    }

    /** Returns the average of the encoder positions reported by the motors */
    public int getAvgEncoderPos() {
        return (
                leftFront.getCurrentPosition() +
                rightFront.getCurrentPosition() +
                leftBack.getCurrentPosition() +
                rightBack.getCurrentPosition()
        ) / 4;
    }

    /** Returns the average of the velocities reported by the motors */
    public double getAvgVelocity() {
        return (
                leftFront.getVelocity() +
                rightFront.getVelocity() +
                leftBack.getVelocity() +
                rightBack.getVelocity()
        ) / 4;
    }

    /** The normal DcMotor function but applied to all motors */
    public void setMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    /** The normal DcMotorEx function but applied to all motors */
    public void setVelocity(double velocity) {
        velocity = accelLimiter.request(velocity, getAvgVelocity(), stopwatch.milliseconds());
        leftFront.setVelocity(velocity);
        rightFront.setVelocity(velocity);
        leftBack.setVelocity(velocity);
        rightBack.setVelocity(velocity);
    }

    /** The normal DcMotor function but applied to all motors */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);

    }
}
