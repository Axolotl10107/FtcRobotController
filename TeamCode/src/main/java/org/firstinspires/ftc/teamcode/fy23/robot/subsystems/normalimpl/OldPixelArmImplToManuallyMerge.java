package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.DigitalDevice;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;
import org.firstinspires.ftc.teamcode.fy23.units.PowerTpSConverter;

/** A normal implementation of {@link PixelArm} featuring acceleration control and hard and soft safety limits. */
public class OldPixelArmImplToManuallyMerge implements PixelArm {

    private DcMotorEx pivotMotor;
    private DcMotorEx elevatorMotor;

    private AccelLimiter pivotAccelLimiter;
    private AccelLimiter elevatorAccelLimiter;

    private PowerTpSConverter pivotConverter;
    private PowerTpSConverter elevatorConverter;

    private int pivotStoppingDistanceAtHalfPower;
    private int pivotStoppingDistanceAtFullPower;
    private int elevatorStoppingDistanceAtHalfPower;
    private int elevatorStoppingDistanceAtFullPower;

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
    private boolean killPivotMotorLatch;
    private double elevatorPower;
    private boolean killElevatorMotorLatch;

    private ElapsedTime stopwatch;

    public OldPixelArmImplToManuallyMerge(Parameters parameters) {
        this(parameters, new ElapsedTime());
    }

    public OldPixelArmImplToManuallyMerge(Parameters parameters, ElapsedTime stopwatch) {
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
        pivotConverter = parameters.pivotPowerTpSConverter;
        pivotTicksPerDegree = parameters.pivotTicksPerDegree;

        double originalPivotMaxAccel = pivotAccelLimiter.getMaxAccel();
        double orignalPivotMaxDeltaVEachLoop = pivotAccelLimiter.getMaxDeltaVEachLoop();
        int convertedPower = pivotConverter.powerToTpS(1.0);
        // work in ticks per second for a little bit
        pivotAccelLimiter.setParameters(convertedPower, convertedPower * orignalPivotMaxDeltaVEachLoop);
        pivotStoppingDistanceAtHalfPower = (int) pivotAccelLimiter.stoppingDistance(pivotConverter.powerToTpS(0.5), 1000);
        pivotAccelLimiter.reset();
        pivotStoppingDistanceAtFullPower = (int) pivotAccelLimiter.stoppingDistance(convertedPower, 1000);
        System.out.println(convertedPower);
        System.out.println(pivotStoppingDistanceAtHalfPower);
        System.out.println(pivotStoppingDistanceAtFullPower);
        // go back to motor power
        pivotAccelLimiter.setParameters(originalPivotMaxAccel, orignalPivotMaxDeltaVEachLoop);

        elevatorAccelLimiter = parameters.elevatorAccelLimiter;
        elevatorConverter = parameters.elevatorPowerTpSConverter;
        elevatorStoppingDistanceAtHalfPower = elevatorConverter.powerToTpS(0.5);
        elevatorStoppingDistanceAtFullPower = elevatorConverter.powerToTpS(1.0);
        elevatorTicksPerMillimeter = parameters.elevatorTicksPerMillimeter;

        this.stopwatch = stopwatch;
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
    public double getPivotVelocity() {
        return 0;
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
        elevatorPower = power;
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
        }
        killPivotMotorLatch = true;
    }

    private void updatePivotPower() {
        double requestedPivotPower = setPivotPower;
//        System.out.println(requestedPivotPower);

        // hard limits
        if (pivotLowerLimitSwitch.isActive()) {
            killPivotMotor();
            requestedPivotPower = Math.max(setPivotPower, 0);
        } else if (pivotUpperLimitSwitch.isActive()) {
            killPivotMotor();
            requestedPivotPower = Math.min(setPivotPower, 0);

            // soft limits
        } else if (getPivotPosition() <= pivotLowerLimit) {
//            System.out.println("Soft lower limit tripped");
            killPivotMotor();
            requestedPivotPower = Math.max(setPivotPower, 0);
        } else if (getPivotPosition() >= pivotUpperLimit) {
            killPivotMotor();
            requestedPivotPower = Math.min(setPivotPower, 0);

            // stopping distances
        } else {
            killPivotMotorLatch = false;
//            System.out.println("Doing stopping distances");
            int stoppingDistance;
            // choose the correct stopping distance...
            if (getPivotPower() <= 0.5) {
                stoppingDistance = pivotStoppingDistanceAtHalfPower;
            } else {
                stoppingDistance = pivotStoppingDistanceAtFullPower;
            }

            // ... then check our position against it
            if (getPivotPosition() - pivotLowerLimit < stoppingDistance) {
                requestedPivotPower = Math.max(setPivotPower, 0);
            } else if (pivotUpperLimit - getPivotPosition() < stoppingDistance) {
                requestedPivotPower = Math.min(setPivotPower, 0);
            }
        }

        pivotMotor.setPower(pivotAccelLimiter.requestVel(requestedPivotPower, getPivotPower(), stopwatch.seconds()));
    }

    private void updateElevatorPower() {
        double currentElevatorPower = getElevatorPower();
        if (!elevatorUpperLimitSwitch.isActive() && !elevatorLowerLimitSwitch.isActive()) {
            if (currentElevatorPower > 0) {
                // if moving
                int currentPos = elevatorMotor.getCurrentPosition();
                if (currentElevatorPower > 0.5) {
                    // if above half power
                    if ((elevatorUpperLimit - currentPos) < elevatorStoppingDistanceAtFullPower) {
                        elevatorPower = Math.min(elevatorPower, 0);
                    } else if (currentPos < (elevatorLowerLimit + elevatorStoppingDistanceAtFullPower)) {
                        elevatorPower = Math.max(0, elevatorPower);
                    }
                } else {
                    // if below half power but still moving
                    if ((elevatorUpperLimit - currentPos) < elevatorStoppingDistanceAtHalfPower) {
                        elevatorPower = Math.min(elevatorPower, 0);
                    } else if (currentPos < (elevatorLowerLimit + elevatorStoppingDistanceAtHalfPower)) {
                        elevatorPower = Math.max(0, elevatorPower);
                    }
                }
            }
        } else {
            // a limit switch is pressed
            elevatorMotor.setPower(0); // kill power now, ask questions later
            if (elevatorUpperLimitSwitch.isActive()) {
                elevatorPower = Math.min(elevatorPower, 0);
            } else {
                elevatorPower = Math.max(0, elevatorPower);
            }
        }
        elevatorMotor.setPower(elevatorAccelLimiter.requestVel(elevatorPower, currentElevatorPower, stopwatch.seconds()));
    }

    @Override
    /** Called by robot.update(). You do not need to call this method. */
    public void update() {
        updatePivotPower();
        updateElevatorPower();
    }

}
