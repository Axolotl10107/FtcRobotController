package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** A normal implementation of {@link MecanumDrive}. Normalize a DTS before passing it in for desirable results. */
public class MecanumDriveImpl implements MecanumDrive {

    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;

    public AccelLimiter accelLimiter;
    public ElapsedTime stopwatch;

    public MecanumDriveImpl(MecanumDrive.Parameters parameters, HardwareMap hardwareMap) {
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
    @Override
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
    @Override
    public void setMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    /** The normal DcMotorEx function but applied to all motors */
    public void setVelocity(double velocity) {
        velocity = accelLimiter.requestVel(velocity, getAvgVelocity(), stopwatch.milliseconds());
        leftFront.setVelocity(velocity);
        rightFront.setVelocity(velocity);
        leftBack.setVelocity(velocity);
        rightBack.setVelocity(velocity);
    }

    /** The normal DcMotor function but applied to all motors */
    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);

    }

    @Override
    public void update() {

    }

}
