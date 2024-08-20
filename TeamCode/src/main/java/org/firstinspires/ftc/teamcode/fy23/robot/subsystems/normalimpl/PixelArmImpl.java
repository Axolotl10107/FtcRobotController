package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.DigitalDevice;
import org.firstinspires.ftc.teamcode.fy23.units.PowerTpSConverter;

/** A normal implementation of {@link org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm} featuring acceleration control and hard and soft safety limits. */
public class PixelArmImpl implements org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm {

    private DcMotorEx pivotMotor;
    private DcMotorEx elevatorMotor;

    private AccelLimiter pivotAccelLimiter;
    private AccelLimiter elevatorAccelLimiter;

//    private PowerTpSConverter pivotConverter;
//    private PowerTpSConverter elevatorConverter;
//
//    private int pivotStoppingDistanceAtHalfPower;
//    private int pivotStoppingDistanceAtFullPower;
//    private int elevatorStoppingDistanceAtHalfPower;
//    private int elevatorStoppingDistanceAtFullPower;

    private int pivotUpperLimit;
    private int pivotLowerLimit;
    private DigitalDevice pivotUpperLimitSwitch;
    private DigitalDevice pivotLowerLimitSwitch;

    private int elevatorUpperLimit;
    private int elevatorLowerLimit;
    private DigitalDevice elevatorUpperLimitSwitch;
    private DigitalDevice elevatorLowerLimitSwitch;

    private double pivotTicksPerDegree;
    private double elevatorTicksPerMillimeter;

    private double setPivotPower;
    private double setElevatorPower;

    private ElapsedTime stopwatch;

    private boolean killPivotMotorLatch = false;
    private double maxPivotRecoveryPower;
    private boolean killElevatorMotorLatch = false;
    private double maxElevatorRecoveryPower;

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
//        pivotConverter = parameters.pivotPowerTpSConverter;
//        pivotStoppingDistanceAtHalfPower = pivotConverter.powerToTpS(0.5);
//        pivotStoppingDistanceAtFullPower = pivotConverter.powerToTpS(1.0);
        pivotTicksPerDegree = parameters.pivotTicksPerDegree;
        maxPivotRecoveryPower = parameters.maxPivotRecoveryPower;

        elevatorAccelLimiter = parameters.elevatorAccelLimiter;
//        elevatorConverter = parameters.elevatorPowerTpSConverter;
//        elevatorStoppingDistanceAtHalfPower = elevatorConverter.powerToTpS(0.5);
//        elevatorStoppingDistanceAtFullPower = elevatorConverter.powerToTpS(1.0);
        elevatorTicksPerMillimeter = parameters.elevatorTicksPerMillimeter;
        maxElevatorRecoveryPower = parameters.maxElevatorRecoveryPower;

        stopwatch = parameters.stopwatch;
    }


    @Override
    public void setPivotAngle(AngleUnit unit, double angle) {
        double angleInDegrees = unit.toDegrees(angle);
        int angleInTicks = (int) (pivotLowerLimit + (pivotTicksPerDegree * angleInDegrees));
        int applyPos = Range.clip(angleInTicks, pivotLowerLimit, pivotUpperLimit);
        pivotMotor.setTargetPosition(applyPos);
        // we don't need to worry about power or limits / AccelLimiter; that's handled by setPower() methods and update(), respectively
    }

    @Override
    public void setPivotPower(double power) {
        setPivotPower = power;
    }

    @Override
    public double getPivotPower() {
        return pivotMotor.getPower();
    }

    @Override
    public int getPivotPosition() {
        return pivotMotor.getCurrentPosition();
    }


    @Override
    public void setElevatorDistance(double distance) {
        int distanceInTicks = (int) (distance * elevatorTicksPerMillimeter);
        int applyPos = Range.clip(distanceInTicks, elevatorLowerLimit, elevatorUpperLimit);
        elevatorMotor.setTargetPosition(applyPos);
    }

    @Override
    public void setElevatorPower(double power) {
        setElevatorPower = power;
    }

    @Override
    public double getElevatorPower() {
        return elevatorMotor.getPower();
    }

    @Override
    public int getElevatorPosition() {
        return elevatorMotor.getCurrentPosition();
    }

    private void killPivotMotor() {
        if (!killPivotMotorLatch) {
            pivotMotor.setPower(0);
            pivotAccelLimiter.reset();
            killPivotMotorLatch = true;
        }
    }

    private void handlePivotHitUpperLimit(double requestedPower) {
        killPivotMotor();
        double safeRequest = Range.clip(requestedPower, -maxPivotRecoveryPower, 0);
        if (safeRequest <= 0) {
            double limited = pivotAccelLimiter.requestVel(safeRequest, getPivotPower(), stopwatch.seconds());
            pivotMotor.setPower(safeRequest);
            if (pivotMotor.getCurrentPosition() < pivotUpperLimit) {
                killPivotMotorLatch = false;
            }
        }
    }

    private void handlePivotHitLowerLimit(double requestedPower) {
        killPivotMotor();
        double safeRequest = Range.clip(requestedPower, 0, maxPivotRecoveryPower);
        if (safeRequest >= 0) {
            double limited = pivotAccelLimiter.requestVel(safeRequest, getPivotPower(), stopwatch.seconds());
            pivotMotor.setPower(safeRequest);
            if (pivotMotor.getCurrentPosition() > pivotLowerLimit) {
                killPivotMotorLatch = false;
            }
        }
    }

    private void handlePivotHitUpperSD(double requestedPower) {
        double safeRequest = Range.clip(requestedPower, -maxPivotRecoveryPower, 0);
        double limited = pivotAccelLimiter.requestVel(safeRequest, getPivotPower(), stopwatch.seconds());
        pivotMotor.setPower(limited);
    }

    private void handlePivotHitLowerSD(double requestedPower) {
        double safeRequest = Range.clip(requestedPower, 0, maxPivotRecoveryPower);
        double limited = pivotAccelLimiter.requestVel(safeRequest, getPivotPower(), stopwatch.seconds());
        pivotMotor.setPower(limited);
    }

    private void updatePivotPower() {
        // checks in a sequence: hard limits, then soft limits, then stopping distances, then apply power as normal

        // limit switches

        if (pivotUpperLimitSwitch.isActive()) {
            System.out.println("Upper limit switch activated");
            handlePivotHitUpperLimit(setPivotPower);
        } else if (pivotLowerLimitSwitch.isActive()) {
            System.out.println("Lower limit switch activated");
            handlePivotHitLowerLimit(setPivotPower);
        } else {

            // soft limits

            int currentPos = getPivotPosition();
            if (currentPos > pivotUpperLimit) {
                System.out.println("Upper soft limit hit");
                handlePivotHitUpperLimit(setPivotPower);
            } else if (currentPos < pivotLowerLimit) {
                System.out.println("Lower soft limit hit");
                handlePivotHitLowerLimit(setPivotPower);
            } else {

                // stopping distances

//                double stoppingDist = pivotAccelLimiter.stoppingDistance(getPivotPower(), 1000);
//                if (currentPos > (pivotUpperLimit - stoppingDist)) {
//                    System.out.println("Upper stopping distance reached");
//                    handlePivotHitUpperSD(setPivotPower);
//                } else if (currentPos > (pivotLowerLimit + stoppingDist)) {
//                    System.out.println("Lower stopping distance reached");
//                    handlePivotHitLowerSD(setPivotPower);
                // } else {

                // it's safe to go, so run through the AccelLimiter as usual
                System.out.println("No safety measures activated, so proceeding normally");
                double limited = pivotAccelLimiter.requestVel(setPivotPower, getPivotPower(), stopwatch.seconds());
                pivotMotor.setPower(limited);
                //}
            }
        }

        // Old and dangerous stuff - this didn't work last time we tried it...
//        double requestedPivotPower = setPivotPower;
//
//        // hard limits
//        if (pivotLowerLimitSwitch.isActive()) {
//            killPivotMotor();
//            requestedPivotPower = Math.max(setPivotPower, 0);
//        } else if (pivotUpperLimitSwitch.isActive()) {
//            killPivotMotor();
//            requestedPivotPower = Math.min(setPivotPower, 0);
//
//        // soft limits
//        } else if (getPivotPosition() <= pivotLowerLimit) {
//            killPivotMotor();
//            requestedPivotPower = Math.max(setPivotPower, 0);
//        } else if (getPivotPosition() >= pivotUpperLimit) {
//            killPivotMotor();
//            requestedPivotPower = Math.min(setPivotPower, 0);
//
//        // stopping distances
//        } else {
//            int stoppingDistance;
//            // choose the correct stopping distance...
//            if (getPivotPower() <= 0.5) {
//                stoppingDistance = pivotStoppingDistanceAtHalfPower;
//            } else {
//                stoppingDistance = pivotStoppingDistanceAtFullPower;
//            }
//
//            // ... then check our position against it
//            if (getPivotPosition() - pivotLowerLimit < stoppingDistance) {
//                requestedPivotPower = Math.max(setPivotPower, 0);
//            } else if (pivotUpperLimit - getPivotPosition() < stoppingDistance) {
//                requestedPivotPower = Math.min(setPivotPower, 0);
//            }
//
//            // ... then check our position against it
//            if (getPivotPosition() - pivotLowerLimit < stoppingDistance) {
//                requestedPivotPower = Math.max(setPivotPower, 0);
//            } else if (pivotUpperLimit - getPivotPosition() < stoppingDistance) {
//                requestedPivotPower = Math.min(setPivotPower, 0);
//            }
//        }
//
//        pivotMotor.setPower(pivotAccelLimiter.requestVel(requestedPivotPower, getPivotPosition(), stopwatch.seconds()));
    }

    private void killElevatorMotor() {
        elevatorMotor.setPower(0);
        elevatorAccelLimiter.reset();
    }

    private void updateElevatorPower() {
        double requestedElevatorPower = setElevatorPower;

        // hard limits
        if (elevatorLowerLimitSwitch.isActive()) {
            killElevatorMotor();
            requestedElevatorPower = Math.max(setElevatorPower, 0);
        } else if (elevatorUpperLimitSwitch.isActive()) {
            killElevatorMotor();
            requestedElevatorPower = Math.min(setElevatorPower, 0);

            // soft limits
        } else if (getElevatorPosition() <= elevatorLowerLimit) {
            killElevatorMotor();
            requestedElevatorPower = Math.max(setElevatorPower, 0);
        } else if (getElevatorPosition() >= elevatorUpperLimit) {
            killElevatorMotor();
            requestedElevatorPower = Math.min(setElevatorPower, 0);

            // stopping distances
        }// else {
//            int stoppingDistance;
//            // choose the correct stopping distance...
//            if (getElevatorPower() <= 0.5) {
//                stoppingDistance = elevatorStoppingDistanceAtHalfPower;
//            } else {
//                stoppingDistance = elevatorStoppingDistanceAtFullPower;
//            }

        // ... then check our position against it
//            if (getElevatorPosition() - elevatorLowerLimit < stoppingDistance) {
//                requestedElevatorPower = Math.max(setElevatorPower, 0);
//            } else if (elevatorUpperLimit - getElevatorPosition() < stoppingDistance) {
//                requestedElevatorPower = Math.min(setElevatorPower, 0);
//            }
        // }

        elevatorMotor.setPower(elevatorAccelLimiter.requestVel(requestedElevatorPower, getElevatorPosition(), stopwatch.seconds()));
    }

    @Override
    /** Called by robot.update(). You do not need to call this method. */
    public void update() {
        updatePivotPower();
        updateElevatorPower();
    }

}
