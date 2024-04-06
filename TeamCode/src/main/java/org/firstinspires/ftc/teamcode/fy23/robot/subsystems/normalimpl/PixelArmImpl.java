package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.DigitalDevice;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;
import org.firstinspires.ftc.teamcode.fy23.units.PowerTpSConverter;

/** A normal implementation of {@link PixelArm} featuring acceleration control and hard and soft safety limits. */
public class PixelArmImpl implements PixelArm {

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

    private double pivotPower;
    private double elevatorPower;

    private ElapsedTime stopwatch;

    public PixelArmImpl(PixelArm.Parameters parameters, HardwareMap hardwareMap) {
        this(parameters, hardwareMap, new ElapsedTime());
    }

    public PixelArmImpl(PixelArm.Parameters parameters, HardwareMap hardwareMap, ElapsedTime stopwatch) {
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
        pivotStoppingDistanceAtHalfPower = pivotConverter.powerToTpS(0.5);
        pivotStoppingDistanceAtFullPower = pivotConverter.powerToTpS(1.0);
        pivotTicksPerDegree = parameters.pivotTicksPerDegree;

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
        pivotPower = power;
    }

    @Override
    public double getPivotPower() {
        return pivotMotor.getPower();
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

    private void updatePivotPower() {
        double currentPivotPower = getPivotPower();
        if (!pivotUpperLimitSwitch.isActive() && !pivotLowerLimitSwitch.isActive()) {
            if (currentPivotPower > 0) {
                // if moving
                int currentPos = pivotMotor.getCurrentPosition();
                if (currentPivotPower > 0.5) {
                    // if above half power
                    if ((pivotUpperLimit - currentPos) < pivotStoppingDistanceAtFullPower) {
                        pivotPower = Math.min(pivotPower, 0);
                    } else if (currentPos < (pivotLowerLimit + pivotStoppingDistanceAtFullPower)) {
                        pivotPower = Math.max(0, pivotPower);
                    }
                } else {
                    // if below half power but still moving
                    if ((pivotUpperLimit - currentPos) < pivotStoppingDistanceAtHalfPower) {
                        pivotPower = Math.min(pivotPower, 0);
                    } else if (currentPos < (pivotLowerLimit + pivotStoppingDistanceAtHalfPower)) {
                        pivotPower = Math.max(0, pivotPower);
                    }
                }
            }
        } else {
            // a limit switch is pressed
            pivotMotor.setPower(0); // kill power now, ask questions later
            if (pivotUpperLimitSwitch.isActive()) {
                pivotPower = Math.min(pivotPower, 0);
            } else {
                pivotPower = Math.max(0, pivotPower);
            }
        }
        pivotMotor.setPower(pivotAccelLimiter.requestVel(pivotPower, currentPivotPower, stopwatch.seconds()));
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
