package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** Represents a "mecanum" drive motor layout. Pass in a DTS with the intended movement of the drivebase, and the
 * implementation will handle mapping that to the individual motors. */
public interface MecanumDrive {

    /** Contains motor names and settings - usually part of a set of Robot parameters */
    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;

        /** maximum individual motor acceleration, in power per second
         * (power loosely represents velocity) */
        public double maxMotorAccel;

        /** prevents jerking - in power */
        public double maxDeltaVEachLoop;

        /** The name of the motor on the left front corner of the drivebase */
        public String leftFrontName;

        /** Direction motor spins when positive power is applied - to drive the motor "backwards",
         * do not set this to reverse! Set the power to a negative value. What "FORWARD" or "REVERSE" is depends on your
         * motors and how you've laid out your robot, so determine that experimentally. */
        public DcMotor.Direction leftFrontDirection;

        public String rightFrontName;
        public DcMotor.Direction rightFrontDirection;

        public String leftBackName;
        public DcMotor.Direction leftBackDirection;

        public String rightBackName;
        public DcMotor.Direction rightBackDirection;

        /** Applies to all motors */
        public DcMotor.RunMode runMode;

        /** Applies to all motors */
        public DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    }

    /** Apply motor powers from a DTS (Drive-Turn-Strafe).
     * @param dts The DTS to apply. Normalize it before passing it in for desirable behavior. */
    void applyDTS(DTS dts);

    /** The usual DcMotor method, but applied to all four motors.
     * @param runMode The RunMode to set */
    void setMode(DcMotor.RunMode runMode);

    /** The usual DcMotor method, but applied to all four motors.
     * @param behavior The ZeroPowerBehavior to set */
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior);

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
