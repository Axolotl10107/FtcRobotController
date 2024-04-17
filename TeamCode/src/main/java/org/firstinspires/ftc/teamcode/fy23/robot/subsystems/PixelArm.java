package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.units.PowerTpSConverter;

/** Represents the combination pivot (tilt, really) and elevator mechanism and allows both to be controlled by setting
 * their powers independently or by specifying a point on the planar region containing all possible points that this
 * mechanism can reach. */
public interface PixelArm {

    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;
        /** The pivot motor object, already grabbed from the HardwareMap (or pass in a MockDcMotorEx for testing) */
        public DcMotorEx pivotMotor;
        /** The elevator motor object, already grabbed from the HardwareMap (or pass in a MockDcMotorEx for testing) */
        public DcMotorEx elevatorMotor;
        /** Pass in an AccelLimiter object that has already been instantiated with the correct parameters for your motor. */
        public AccelLimiter pivotAccelLimiter;
        /** Pass in a PowerTpSConverter (an implementation of your choice) that has already been instantiated with the
         * correct parameters for your motor. */
        public PowerTpSConverter pivotPowerTpSConverter;
        /** How many encoder ticks are traveled by the pivot motor per degree of pivot arm rotation */
        public double pivotTicksPerDegree;
        /** The upper limit of the pivot motor's range in encoder ticks */
        public int pivotUpperLimit;
        /** The lower limit of the pivot motor's range in encoder ticks */
        public int pivotLowerLimit;
        /** Pass in a DigitalDevice object (an implementation of your choice) to represent a limit switch that is
         * activated when the pivot arm reaches its maximum position (in encoder ticks!). */
        public DigitalDevice pivotUpperLimitSwitch;
        /** Pass in a DigitalDevice object (an implementation of your choice) to represent a limit switch that is
         * activated when the pivot arm reaches its minimum position (in encoder ticks!). */
        public DigitalDevice pivotLowerLimitSwitch;
        /** Pass in an AccelLimiter object that has already been instantiated with the correct parameters for your motor. */
        public AccelLimiter elevatorAccelLimiter;
        /** Pass in a PowerTpSConverter object (an implementation of your choice) that has already been instantiated
         * with the correct parameters for your motor. */
        public PowerTpSConverter elevatorPowerTpSConverter;
        /** How many encoder ticks are traveled by the elevator motor per degree of elevator travel */
        public double elevatorTicksPerMillimeter;
        /** The upper limit of the elevator motor's range in encoder ticks */
        public int elevatorUpperLimit;
        /** The lower limit of the elevator motor's range in encoder ticks */
        public int elevatorLowerLimit;
        /** Pass in a DigitalDevice object (an implementation of your choice) to represent a limit switch that is
         * activated when the elevator reaches its maximum position (in encoder ticks!). */
        public DigitalDevice elevatorUpperLimitSwitch;
        /** Pass in a DigitalDevice object (an implementation of your choice) to represent a limit switch that is
         * activated when the elevator reaches its minimum position (in encoder ticks!). */
        public DigitalDevice elevatorLowerLimitSwitch;
    }

    /** Set the target position of the pivot motor to an angle. Safety limits apply.
     * @param unit See {@link AngleUnit}.
     * @param angle in the AngleUnit you set */
    void setPivotAngle(AngleUnit unit, double angle);
    /** Set the power of the pivot motor. Important when setting an angle - this works like setPower() does on a normal
     * motor in RUN_TO_POSITION mode.
     * @param power from 0 to 1 just like setPower() on a normal motor */
    void setPivotPower(double power);
    /** Get the current power of the pivot motor. */
    double getPivotPower();
    /** Get the current position of the pivot motor. */
    int getPivotPosition();

    /** Set the target position of the elevator motor to a distance from the fully retracted position.
     * @param distance in millimeters */
    void setElevatorDistance(double distance);
    /** Set the power of the elevator motor. Important when setting a distance - this works like setPower() does on a
     * normal motor in RUN_TO_POSITION mode.
     * @param power from 0 to 1, just like setPower() on a normal motor */
    void setElevatorPower(double power);
    /** Get the current power of the elevator motor. */
    double getElevatorPower();
    /** Get the current position of the elevator motor. */
    int getElevatorPosition();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
