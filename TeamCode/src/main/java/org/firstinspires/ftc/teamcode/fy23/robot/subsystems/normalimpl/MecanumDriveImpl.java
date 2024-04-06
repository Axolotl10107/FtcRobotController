package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

import java.util.Arrays;
import java.util.List;

/** A normal implementation of {@link MecanumDrive}. */
public class MecanumDriveImpl implements MecanumDrive {

    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;

    private AccelLimiter accelLimiter;
    private ElapsedTime stopwatch;

    /** Pass in an ElapsedTime. Useful for UnitTests, which can pass in a MockElapsedTime.
     * @param parameters Passed in by the Robot. Your OpMode doesn't need to worry about this.
     * @param stopwatch ElapsedTime to be used for acceleration control */
    public MecanumDriveImpl(MecanumDrive.Parameters parameters, ElapsedTime stopwatch) {
        leftFront = parameters.leftFrontMotor;
        rightFront = parameters.rightFrontMotor;
        leftBack = parameters.leftBackMotor;
        rightBack = parameters.rightBackMotor;

        try {
            setMode(parameters.runMode);
        } catch (Exception x) {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // if the RunMode wasn't set, pick a default
        }

        try {
            setZeroPowerBehavior(parameters.zeroPowerBehavior);
        } catch (Exception x) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        accelLimiter = parameters.accelLimiter;
        this.stopwatch = stopwatch;
    }

    /** Creates a normal ElapsedTime. Good for use in OpModes.
     * @param parameters Passed in by the Robot. Your OpMode doesn't need to worry about this. */
    public MecanumDriveImpl(MecanumDrive.Parameters parameters) {
        this(parameters, new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS));
    }

    /** Takes these components as motor powers and limits the acceleration.
     * @param drive Forwards power
     * @param turn Turning power - positive is counterclockwise!
     * @param strafe Strafing (sideways) power - positive is to the right */
    public void applyDTS(double drive, double turn, double strafe) {
        double newLF = drive - turn + strafe;
        double newRF = drive + turn - strafe;
        double newLB = drive - turn - strafe;
        double newRB = drive + turn + strafe;
        double currentLF = leftFront.getPower();
        double currentRF = rightFront.getPower();
        double currentLB = leftBack.getPower();
        double currentRB = rightBack.getPower();
        double requestedDeltaLF = newLF - currentLF;
        double requestedDeltaRF = newRF - currentRF;
        double requestedDeltaLB = newLB - currentLB;
        double requestedDeltaRB = newRB - currentRB;
        List<Double> requestList = Arrays.asList(requestedDeltaLF, requestedDeltaRF, requestedDeltaLB, requestedDeltaRB);

        List<Double> returnList = accelLimiter.requestDeltaVelOnN(requestList, stopwatch.seconds());
        newLF = currentLF + returnList.get(0);
        newRF = currentRF + returnList.get(1);
        newLB = currentLB + returnList.get(2);
        newRB = currentRB + returnList.get(3);
        leftFront.setPower(newLF);
        rightFront.setPower(newRF);
        leftBack.setPower(newLB);
        rightBack.setPower(newRB);

        System.out.println(String.format("Requested deltaVels: | {%f} | {%f} | {%f} | {%f}", requestedDeltaLF, requestedDeltaRF, requestedDeltaLB, requestedDeltaRB));
        System.out.println(String.format("Motor Powers: | {%f} | {%f} | {%f} | {%f}", newLF, newRF, newLB, newRB));
    }

    /** Takes a DTS (Drive-Turn-Strafe) of motor powers.
     * @param dts The DTS to apply */
    @Override
    public void applyDTS(DTS dts) { // method overloading
        System.out.println("Ooh, fancy!");
        applyDTS(dts.drive, dts.turn, dts.strafe);
    }

    /** Returns the average of the encoder positions reported by the motors */
    @Deprecated
    public int getAvgEncoderPos() {
        return (
                leftFront.getCurrentPosition() +
                        rightFront.getCurrentPosition() +
                        leftBack.getCurrentPosition() +
                        rightBack.getCurrentPosition()
        ) / 4;
    }

    /** Returns the average of the velocities reported by the motors */
    @Deprecated
    public double getAvgVelocity() {
        return (
                leftFront.getVelocity() +
                        rightFront.getVelocity() +
                        leftBack.getVelocity() +
                        rightBack.getVelocity()
        ) / 4;
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    @Deprecated
    public void setVelocity(double velocity) {
        velocity = accelLimiter.requestVel(velocity, getAvgVelocity(), stopwatch.milliseconds());
        leftFront.setVelocity(velocity);
        rightFront.setVelocity(velocity);
        leftBack.setVelocity(velocity);
        rightBack.setVelocity(velocity);
    }

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
