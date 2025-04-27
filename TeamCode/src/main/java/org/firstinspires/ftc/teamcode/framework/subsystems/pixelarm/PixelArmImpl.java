package org.firstinspires.ftc.teamcode.framework.subsystems.pixelarm;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.framework.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.framework.processors.TunablePID;
import org.firstinspires.ftc.teamcode.framework.subsystems.digitaldevice.DigitalDevice;
import org.firstinspires.ftc.teamcode.framework.units.PIDConsts;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;

/** A normal implementation of {@link PixelArm} featuring
 * acceleration control and hard and soft safety limits.
 * Note that this now uses velocity, not power, under the hood, so motor behavior may be just a bit different (and
 * hopefully better) than what you're used to.
 * <b>A bug currently exists in either this or AccelLimiter such that maxAccel seems to be divided by 10.
 * Workaround: If your max. velocity should be 800 t/s, set 8000 t/s in the AccelLimiters.</b>
 *
 * <hr>
 *
 * */
@Deprecated
public class PixelArmImpl implements PixelArm {

    private final DcMotorEx pivotMotor;
    private final DcMotorEx elevatorMotor;

    private final AccelLimiter pivotAccelLimiter;
    private final AccelLimiter elevatorAccelLimiter;

    private final int pivotUpperLimit;
    private final int pivotLowerLimit;
    private final DigitalDevice pivotUpperLimitSwitch;
    private final DigitalDevice pivotLowerLimitSwitch;

    private int elevatorUpperLimit;
    private int elevatorLowerLimit;
    private final DigitalDevice elevatorUpperLimitSwitch;
    private final DigitalDevice elevatorLowerLimitSwitch;

    private final double pivotTicksPerDegree;
    private final double elevatorTicksPerMillimeter;

    private double setPivotVelocity;
    private final int maxPivotVelocity;

    private int setElevatorVelocity;
    private final int maxElevatorVelocity;
    private double setElevatorPower;

    private final ElapsedTime stopwatch;

    private boolean killPivotMotorLatch = false;
    private final double maxPivotRecoveryPower;
    private boolean killElevatorMotorLatch = false;
    private final double maxElevatorRecoveryPower;

    // TODO: TESTING
    private final PIDConsts pivotPIDConsts = new PIDConsts(0.00001, 0.00001, 0.001, 0.000);
    private double heartbeat = 0.0001;
    private final TunablePID pivotPID = new TunablePID(pivotPIDConsts);

    @SuppressWarnings("FieldCanBeLocal")
    private final boolean useSdkPid = false;

    public PixelArmImpl(Parameters parameters) {
        pivotMotor = parameters.pivotMotor;
        elevatorMotor = parameters.elevatorMotor;

        pivotUpperLimit = parameters.pivotUpperLimit;
        pivotLowerLimit = parameters.pivotLowerLimit;
        pivotUpperLimitSwitch = parameters.pivotUpperLimitSwitch;
        pivotLowerLimitSwitch = parameters.pivotLowerLimitSwitch;

        elevatorUpperLimit = parameters.elevatorUpperLimit;
        elevatorLowerLimit = parameters.elevatorLowerLimit;
        elevatorUpperLimitSwitch = parameters.elevatorUpperLimitSwitch;
        elevatorLowerLimitSwitch = parameters.elevatorLowerLimitSwitch;

        pivotAccelLimiter = parameters.pivotAccelLimiter;
        pivotTicksPerDegree = parameters.pivotTicksPerDegree;
        maxPivotRecoveryPower = parameters.maxPivotRecoveryPower;
        maxPivotVelocity = parameters.maxPivotVelocity;

        elevatorAccelLimiter = parameters.elevatorAccelLimiter;
        elevatorTicksPerMillimeter = parameters.elevatorTicksPerMillimeter;
        maxElevatorRecoveryPower = parameters.maxElevatorRecoveryPower;
        maxElevatorVelocity = parameters.maxElevatorVelocity;

        stopwatch = parameters.stopwatch;
    }


    @Override
    public void setPivotAngle(AngleUnit unit, double angle) {
        double angleInDegrees = unit.toDegrees(angle);
        int angleInTicks = (int) (pivotLowerLimit + Math.round(pivotTicksPerDegree * angleInDegrees));
        int applyPos = Range.clip(angleInTicks, pivotLowerLimit, pivotUpperLimit);
        pivotMotor.setTargetPosition(applyPos);
        // we don't need to worry about power or limits / AccelLimiter; that's handled by setPower() methods and update(), respectively
    }

    @Override
    public void setPivotPower(double power) {
        setPivotVelocity = (int) Math.round(maxPivotVelocity * power);
    }

    @Override
    public double getPivotPower() {
        return pivotMotor.getPower();
    }

    @Override
    public void setPivotVelocity(int velocity) {
        setPivotVelocity = Range.clip(velocity, -maxPivotVelocity, maxPivotVelocity);
    }

    @Override
    public double getPivotVelocity() {
        return pivotMotor.getVelocity();
    }

    @Override
    public int getPivotPosition() {
        return pivotMotor.getCurrentPosition();
    }


    @Override
    public void setElevatorDistance(double distance) {
        int distanceInTicks = (int) Math.round(distance * elevatorTicksPerMillimeter);
        int applyPos = Range.clip(distanceInTicks, elevatorLowerLimit, elevatorUpperLimit);
        elevatorMotor.setTargetPosition(applyPos);
    }

    @Override
    public void setElevatorPower(double power) {
        setElevatorVelocity = (int) Math.round(maxElevatorVelocity * power);
    }

    @Override
    public double getElevatorPower() {
        return elevatorMotor.getPower();
    }

    @Override
    public void setElevatorVelocity(int velocity) {
        elevatorMotor.setVelocity(velocity);
    }

    @Override
    public double getElevatorVelocity() {
        return elevatorMotor.getVelocity();
    }

    @Override
    public int getElevatorPosition() {
        return elevatorMotor.getCurrentPosition();
    }

    private void killPivotMotor() {
        if (!killPivotMotorLatch) {
            pivotMotor.setVelocity(0);
            pivotAccelLimiter.reset();
            killPivotMotorLatch = true;
        }
    }

    private void handlePivotHitUpperLimit(double requestedPower) {
        killPivotMotor();
        double safeRequest = Range.clip(requestedPower, -maxPivotVelocity * maxPivotRecoveryPower, 0);
        if (safeRequest <= 0) {
//            double limited = pivotAccelLimiter.requestVel(safeRequest, getPivotVelocity(), stopwatch.seconds());
            pivotMotor.setVelocity(safeRequest);
            if (pivotMotor.getCurrentPosition() < pivotUpperLimit) {
                killPivotMotorLatch = false;
            }
        }
    }

    private void handlePivotHitLowerLimit(double requestedVelocity) {
        killPivotMotor();
        double safeRequest = Range.clip(requestedVelocity, 0, maxPivotVelocity * maxPivotRecoveryPower);
        if (safeRequest >= 0) {
//            double limited = pivotAccelLimiter.requestVel(safeRequest, getPivotVelocity(), stopwatch.seconds());
            pivotMotor.setVelocity(safeRequest);
            if (pivotMotor.getCurrentPosition() > pivotLowerLimit) {
                killPivotMotorLatch = false;
            }
        }
    }

    private void handlePivotHitUpperSD(double requestedVelocity) {
        double safeRequest = Range.clip(requestedVelocity, -maxPivotVelocity * maxPivotRecoveryPower, 0);
        double limited = pivotAccelLimiter.requestVel(safeRequest, getPivotVelocity(), stopwatch.seconds());
        pivotMotor.setVelocity(limited);
    }

    private void handlePivotHitLowerSD(double requestedPower) {
        double safeRequest = Range.clip(requestedPower, 0, maxPivotVelocity * maxPivotRecoveryPower);
        double limited = pivotAccelLimiter.requestVel(safeRequest, getPivotVelocity(), stopwatch.seconds());
        pivotMotor.setVelocity(limited);
    }

    private void setPivotVelSDK() {
        double limited = pivotAccelLimiter.requestVel(
                setPivotVelocity,                               // newVel
                pivotMotor.getVelocity(),                       // currentVel
                stopwatch.seconds()                             // currentTime
        );
        pivotMotor.setVelocity(limited);
//        System.out.println(pivotMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    //TODO: Testing
    private void setPivotVelTunablePID() {
        double velError = setPivotVelocity - pivotMotor.getVelocity();
        System.out.println(velError);
//        double limitedVelError = pivotAccelLimiter.requestDeltaVel( velError, stopwatch.seconds() );
        double limitedVelError = velError;
        double deltaPower = pivotPID.correctFor( limitedVelError );
        pivotMotor.setPower( pivotMotor.getPower() + deltaPower );
        System.out.printf("velError: {%f}  |  limitedVelError: {%f}%n", velError, limitedVelError);
        System.out.printf("deltaPower: {%f}  |  currentPower: {%f}%n", deltaPower, pivotMotor.getPower());
        System.out.println("-----------------------------------------------");
        TelemetrySingleton.getInstance().addData("Current Integral", pivotPID.currentIntegralValue());
        TelemetrySingleton.getInstance().addData("Velocity Error", velError);
        TelemetrySingleton.getInstance().addData("Limited Velocity Error (PID input)", limitedVelError);
        TelemetrySingleton.getInstance().addData("Delta Power (PID output)", deltaPower);
        TelemetrySingleton.getInstance().addData("Scaled Delta Power (PID output)", deltaPower * maxPivotVelocity);
        TelemetrySingleton.getInstance().addData("Requested Velocity (user input)", setPivotVelocity);
        heartbeat = -heartbeat;
        TelemetrySingleton.getInstance().addData("Heartbeat", heartbeat);
    }

    private void updatePivotPower() {
        // checks in a sequence: hard limits, then soft limits, then stopping distances, then apply power as normal

        // limit switches

        if (pivotUpperLimitSwitch.isActive()) {
            System.out.println("[Pivot] Upper limit switch activated");
            handlePivotHitUpperLimit(setPivotVelocity);
        } else if (pivotLowerLimitSwitch.isActive()) {
            System.out.println("[Pivot] Lower limit switch activated");
            handlePivotHitLowerLimit(setPivotVelocity);
        } else {

            // soft limits

            int currentPos = getPivotPosition();
            if (currentPos > pivotUpperLimit) {
                System.out.println("[Pivot] Upper soft limit hit");
                handlePivotHitUpperLimit(setPivotVelocity);
            } else if (currentPos < pivotLowerLimit) {
                System.out.println("[Pivot] Lower soft limit hit");
                handlePivotHitLowerLimit(setPivotVelocity);
            } else {

                // stopping distances

//                double stoppingDist = pivotAccelLimiter.simpleStoppingDistance(getPivotVelocity() * 10);
//                TelemetrySingleton.getInstance().addData("pivotStoppingDist", stoppingDist);
//                if (currentPos > (pivotUpperLimit - stoppingDist)) {
//                    System.out.println("[Pivot] Upper stopping distance reached");
//                    handlePivotHitUpperSD(setPivotVelocity);
//                } else if (currentPos < (pivotLowerLimit + stoppingDist)) {
//                    System.out.println("[Pivot] Lower stopping distance reached");
//                    handlePivotHitLowerSD(setPivotVelocity);
//                 } else {

//                 it's safe to go, so run through the AccelLimiter as usual
                System.out.println("[Pivot] No safety measures activated, so proceeding normally");
                if (useSdkPid) {
                    setPivotVelSDK();
                } else {
                    setPivotVelTunablePID();
                }
//                }
            }
        }
    }

    private void killElevatorMotor() {
        if (!killElevatorMotorLatch) {
            elevatorMotor.setVelocity(0);
            elevatorAccelLimiter.reset();
            killElevatorMotorLatch = true;
        }
    }

    private void handleElevatorHitUpperLimit(double requestedPower) {
        killElevatorMotor();
        double safeRequest = Range.clip(requestedPower, -maxElevatorVelocity * maxElevatorRecoveryPower, 0);
        if (safeRequest <= 0) {
//            double limited = elevatorAccelLimiter.requestVel(safeRequest, getElevatorVelocity(), stopwatch.seconds());
            elevatorMotor.setVelocity(safeRequest);
            if (elevatorMotor.getCurrentPosition() < elevatorUpperLimit) {
                killElevatorMotorLatch = false;
            }
        }
    }

    private void handleElevatorHitLowerLimit(double requestedVelocity) {
        killElevatorMotor();
        double safeRequest = Range.clip(requestedVelocity, 0, maxElevatorVelocity * maxElevatorRecoveryPower);
        if (safeRequest >= 0) {
//            double limited = elevatorAccelLimiter.requestVel(safeRequest, getElevatorVelocity(), stopwatch.seconds());
            elevatorMotor.setVelocity(safeRequest);
            if (elevatorMotor.getCurrentPosition() > elevatorLowerLimit) {
                killElevatorMotorLatch = false;
            }
        }
    }

    private void handleElevatorHitUpperSD(double requestedVelocity) {
        double safeRequest = Range.clip(requestedVelocity, -maxElevatorVelocity * maxElevatorRecoveryPower, 0);
        double limited = elevatorAccelLimiter.requestVel(safeRequest, getElevatorVelocity(), stopwatch.seconds());
        elevatorMotor.setVelocity(limited);
    }

    private void handleElevatorHitLowerSD(double requestedPower) {
        double safeRequest = Range.clip(requestedPower, 0, maxElevatorVelocity * maxElevatorRecoveryPower);
        double limited = elevatorAccelLimiter.requestVel(safeRequest, getElevatorVelocity(), stopwatch.seconds());
        elevatorMotor.setVelocity(limited);
    }

    private void setElevatorVelSDK() {
        double limited = elevatorAccelLimiter.requestVel(
                setElevatorVelocity,                               // newVel
                (int) Math.round(elevatorMotor.getVelocity()),     // currentVel
                stopwatch.seconds()                                // currentTime
        );
        elevatorMotor.setVelocity(limited);
    }

    private void setElevatorVelTunablePID() {
        // TODO: Implement this!
    }

    private void updateElevatorPower() {
        // checks in a sequence: hard limits, then soft limits, then stopping distances, then apply power as normal

        // limit switches

        if (elevatorUpperLimitSwitch.isActive()) {
            System.out.println("[Elevator] Upper limit switch activated");
            handleElevatorHitUpperLimit(setElevatorVelocity);
        } else if (elevatorLowerLimitSwitch.isActive()) {
            System.out.println("[Elevator] Lower limit switch activated");
            handleElevatorHitLowerLimit(setElevatorVelocity);
        } else {

            // soft limits

            int currentPos = getElevatorPosition();
            if (currentPos > elevatorUpperLimit) {
                System.out.println("[Elevator] Upper soft limit hit");
                handleElevatorHitUpperLimit(setElevatorVelocity);
            } else if (currentPos < elevatorLowerLimit) {
                System.out.println("[Elevator] Lower soft limit hit");
                handleElevatorHitLowerLimit(setElevatorVelocity);
            } else {

                // stopping distances

//                double stoppingDist = elevatorAccelLimiter.stoppingDistance(getElevatorPower(), 1000);
//                if (currentPos > (elevatorUpperLimit - stoppingDist)) {
//                    System.out.println("[Elevator] Upper stopping distance reached");
//                    handleElevatorHitUpperSD(setElevatorPower);
//                } else if (currentPos < (elevatorLowerLimit + stoppingDist)) {
//                    System.out.println("[Elevator] Lower stopping distance reached");
//                    handleElevatorHitLowerSD(setElevatorPower);
                // } else {

                // it's safe to go, so run through the AccelLimiter as usual
//                System.out.println("[Elevator] No safety measures activated, so proceeding normally");
//                if (useSdkPid) {
                if (true) {
                    setElevatorVelSDK();
                } else {
                    setElevatorVelTunablePID();
                }
                //}
            }
        }
    }

    /** Called by robot.update(). You do not need to call this method. */
    @Override
    public void update() {
        updatePivotPower();
        updateElevatorPower();
    }

    public void setElevatorUpperLimit(int i) {
        elevatorUpperLimit = i;
    }

    public void setElevatorLowerLimit(int i) {
        elevatorLowerLimit = i;
    }

}
