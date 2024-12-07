package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.hardwaredevice.BlankMotor;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.digitaldevice.DigitalDeviceBlank;

/**
 * ATTENTION: New for 2024:
 * <br>New parameters:
 * <ul>
 *     <li>elevatorLimitBuffer</li>
 *     <li>elevatorOffsetLength</li>
 * </ul>
 * These, as well as elevator upper and lower limits, are now in <b>inches, measured from the back
 * of the robot.</b> This change was made because of:
 * <ul>
 *     <li>The new game rule this year - our robot cannot be more than 42 inches long in that direction</li>
 *     <li>How the new extension limit code works</li>
 * </ul>
 * <hr><br>
 * Represents the combination pivot and elevator mechanism and allows both to be controlled by setting
 * their powers independently or by setting a pivot angle (degrees) and extension distance (<b>inches</b> - see above).
 * The "setPower()" methods do not actually set power. They set a percentage of the max. velocity.
 * <br><br><hr><br>
 * To work around an unknown bug (presumably in AccelLimiter?), when passing an AccelLimiter into
 * this subsystem (or creating one anywhere), multiply the maxAccel and maxDeltaVEachLoop arguments
 * by 10.
 * */
public interface DoubleArm {

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
        public DcMotorEx pivotMotorLeft = new BlankMotor();
        public DcMotorEx pivotMotorRight = new BlankMotor();
        /** The elevator motor object, already grabbed from the HardwareMap (or pass in a MockDcMotorEx for testing) */
        public DcMotorEx elevatorMotorLeft = new BlankMotor();
        public DcMotorEx elevatorMotorRight = new BlankMotor();


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
        /** How many encoder ticks are traveled by the elevator motor per inch of elevator travel */
        public double elevatorTicksPerInch = 0;
        /** How much extra space in <b>inches</b> to leave in front of the elevator */
        public double elevatorLimitBuffer = 0;
        /** The distance in <b>inches</b> from the back of the robot to the start of the elevator <i>slides</i> (the actual slides, not the motor) */
        public double elevatorOffsetLength = 0;
        /** The upper limit of the elevator motor's range in <b>inches</b>, measured from the back of the robot */
        public int elevatorUpperLimit = 0;
        /** The lower limit of the elevator motor's range in <b>inches</b> */
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
        public int maxElevatorVelocity = 0;
        /** UnitTests can pass in a MockElapsedTime. */
        public ElapsedTime stopwatch = new ElapsedTime();
    }

    /** Set the target position of the pivot motors to an angle. Safety limits apply.
     * @param unit See {@link AngleUnit}.
     * @param angle in the AngleUnit you set */
    void setPivotAngle(AngleUnit unit, double angle);
    /** Set the power of the pivot motors. Important when setting an angle - this works like setPower() does on a normal
     * motor in RUN_TO_POSITION mode.
     * This does not actually set power anymore - this sets the velocity to the max. velocity multiplied by your requested power.
     * @param power from -1 to 1 just like setPower() on a normal motor */
    void setPivotPower(double power);
    /** Get the current average power of the pivot motors. */
    double getPivotPower();
    /** Set the velocity of the pivot motors in ticks per second.
     * @param velocity The velocity to set, in ticks per second */
    void setPivotVelocity(int velocity);
    /** Get the current average velocity of the pivot motors. */
    double getPivotVelocity();
    /** Get the current average position of the pivot motors. */
    int getPivotPosition();

    /** Set the target position of the elevator motors to a distance from the back of the robot.
     * @param distance in inches */
    void setElevatorDistance(double distance);
    /** Set the power of the elevator motors. Important when setting a distance - this works like setPower() does on a
     * normal motor in RUN_TO_POSITION mode.
     * @param power from -1 to 1, just like setPower() on a normal motor */
    void setElevatorPower(double power);
    /** Get the current average power of the elevator motors. */
    double getElevatorPower();
    /** Set the velocity of the elevator motors in ticks per second.
     * @param velocity The velocity to set, in ticks per second */
    void setElevatorVelocity(int velocity);
    /** Get the current average velocity of the elevator motors. */
    double getElevatorVelocity();
    /** Get the current average position of the elevator motors. */
    int getElevatorPosition();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
