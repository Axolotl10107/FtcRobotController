package org.firstinspires.ftc.teamcode.framework.old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.framework.units.DTS;

import java.util.Arrays;
import java.util.List;

/** A normal implementation of {@link MecanumDrive} featuring acceleration control.
 * {@link MecanumDrive} is known to have serious bugs and has been superseded by {@link RRMecanumDrive}.
 * Please use that instead. */
@Deprecated
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

    @Override
    public void applyDTS(DTS dts) { // method overloading
        double newLF = dts.drive - dts.turn + dts.strafe;
        double newRF = dts.drive + dts.turn - dts.strafe;
        double newLB = dts.drive - dts.turn - dts.strafe;
        double newRB = dts.drive + dts.turn + dts.strafe;
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

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
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
