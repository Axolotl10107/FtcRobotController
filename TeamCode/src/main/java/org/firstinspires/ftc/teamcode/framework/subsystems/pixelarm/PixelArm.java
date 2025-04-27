package org.firstinspires.ftc.teamcode.framework.subsystems.pixelarm;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.framework.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice.BlankMotor;
import org.firstinspires.ftc.teamcode.framework.subsystems.digitaldevice.DigitalDevice;
import org.firstinspires.ftc.teamcode.framework.subsystems.digitaldevice.DigitalDeviceBlank;
import org.firstinspires.ftc.teamcode.fy24.subsystems.doublearm.DoubleArm;

/** Represents the combination pivot and elevator mechanism and allows both to be controlled by setting
 * their powers independently or by specifying a point on the planar region containing all possible points that this
 * mechanism can reach.
 * Deprecated because the physical hardware this corresponds to no longer exists (real-world testing is impossible at the moment).
 * A newer generation of it does, driven by {@link DoubleArm} (which is a newer
 * generation of PixelArm).
 * However, the plan is to backport fixes from DoubleArm to this class and
 * instead use a DualMotor adapter (which has yet to be created) to drive the
 * physical DoubleArm. And then DoubleArm will be deprecated instead. */
@Deprecated
public interface PixelArm {

    @Deprecated
    /** You must set some of these if this subsystem is present. */
    class Parameters {
        /** Create a Parameters object and provide parameters that don't have default values.
         * @param present Is this subsystem installed on this robot?
         */
        public Parameters(boolean present) {
            this.present = present;
        }

        /** You already set this in the constructor and cannot set it again. */
        public final boolean present;
        /** The pivot motor object, already grabbed from the HardwareMap (or pass in a MockDcMotorEx for testing) */
        public DcMotorEx pivotMotor = new BlankMotor();
        /** The elevator motor object, already grabbed from the HardwareMap (or pass in a MockDcMotorEx for testing) */
        public DcMotorEx elevatorMotor = new BlankMotor();


        /** Pass in an AccelLimiter object that has already been instantiated with the correct parameters for your motor. */
        public AccelLimiter pivotAccelLimiter = new AccelLimiter(0, 0);
        /** How many encoder ticks are traveled by the pivot motor per degree of pivot arm rotation */
        public double pivotTicksPerDegree = 0;
        /** The upper limit of the pivot motor's range in encoder ticks */
        public int pivotUpperLimit = 0;
        /** The lower limit of the pivot motor's range in encoder ticks */
        public int pivotLowerLimit = 0;
        /** Pass in a DigitalDevice object (an implementation of your choice) to represent a limit switch that is
         * activated when the pivot arm reaches its maximum position (in encoder ticks!). */
        public DigitalDevice pivotUpperLimitSwitch = new DigitalDeviceBlank();
        /** Pass in a DigitalDevice object (an implementation of your choice) to represent a limit switch that is
         * activated when the pivot arm reaches its minimum position (in encoder ticks!). */
        public DigitalDevice pivotLowerLimitSwitch = new DigitalDeviceBlank();
        /** The maximum power to use while the pivot arm has tripped a limit and is returning to a
         * safe position. This is important because acceleration control is not applied at this stage,
         * so a large value here will cause jolts. */
        public double maxPivotRecoveryPower = 0;
        /** The maximum velocity of the pivot motor in ticks per second. */
        public int maxPivotVelocity = 0;


        /** Pass in an AccelLimiter object that has already been instantiated with the correct parameters for your motor. */
        public AccelLimiter elevatorAccelLimiter = new AccelLimiter(0, 0);
        /** How many encoder ticks are traveled by the elevator motor per degree of elevator travel */
        public double elevatorTicksPerMillimeter = 0;
        /** The upper limit of the elevator motor's range in encoder ticks */
        public int elevatorUpperLimit = 0;
        /** The lower limit of the elevator motor's range in encoder ticks */
        public int elevatorLowerLimit = 0;
        /** Pass in a DigitalDevice object (an implementation of your choice) to represent a limit switch that is
         * activated when the elevator reaches its maximum position (in encoder ticks!). */
        public DigitalDevice elevatorUpperLimitSwitch = new DigitalDeviceBlank();
        /** Pass in a DigitalDevice object (an implementation of your choice) to represent a limit switch that is
         * activated when the elevator reaches its minimum position (in encoder ticks!). */
        public DigitalDevice elevatorLowerLimitSwitch = new DigitalDeviceBlank();
        /** The maximum power to use while the elevator has tripped a limit and is returning to a
         * safe position. This is important because acceleration control is not applied at this stage,
         * so a large value here will cause jolts. */
        public double maxElevatorRecoveryPower = 0;
        /** The maximum velocity of the elevator motor in ticks per second. */
        public int maxElevatorVelocity;
        /** UnitTests can pass in a MockElapsedTime. */
        public ElapsedTime stopwatch = new ElapsedTime();
    }

    /** Set the target position of the pivot motor to an angle. Safety limits apply.
     * @param unit See {@link AngleUnit}.
     * @param angle in the AngleUnit you set */
    void setPivotAngle(AngleUnit unit, double angle);
    /** Set the power of the pivot motor. Important when setting an angle - this works like setPower() does on a normal
     * motor in RUN_TO_POSITION mode.
     * This does not actually set power anymore - this sets the velocity to the max. velocity multiplied by your requested power.
     * @param power from -1 to 1 just like setPower() on a normal motor */
    void setPivotPower(double power);
    /** Get the current power of the pivot motor. */
    double getPivotPower();
    /** Set the velocity of the pivot motor in ticks per second.
     * @param velocity The velocity to set, in ticks per second */
    void setPivotVelocity(int velocity);
    /** Get the current velocity of the pivot motor. */
    double getPivotVelocity();
    /** Get the current position of the pivot motor. */
    int getPivotPosition();

    /** Set the target position of the elevator motor to a distance from the fully retracted position.
     * @param distance in millimeters */
    void setElevatorDistance(double distance);
    /** Set the power of the elevator motor. Important when setting a distance - this works like setPower() does on a
     * normal motor in RUN_TO_POSITION mode.
     * @param power from -1 to 1, just like setPower() on a normal motor */
    void setElevatorPower(double power);
    /** Get the current power of the elevator motor. */
    double getElevatorPower();
    /** Set the velocity of the elevator motor in ticks per second.
     * @param velocity The velocity to set, in ticks per second */
    void setElevatorVelocity(int velocity);
    /** Get the current velocity of the elevator motor. */
    double getElevatorVelocity();
    /** Get the current position of the elevator motor. */
    int getElevatorPosition();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
