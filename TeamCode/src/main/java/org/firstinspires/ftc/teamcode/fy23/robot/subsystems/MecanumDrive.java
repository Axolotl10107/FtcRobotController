package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** Represents a "mecanum" drive motor layout. Pass in a DTS with the intended movement of the entire drivebase, and the
 * implementation will handle mapping that to the individual motors. */
public interface MecanumDrive {

    class Parameters {
        public boolean present; /** Is this subsystem installed on this robot? */

        /** max. individual motor acceleration, in power per second
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

        public DcMotor.RunMode runMode; /** Applies to all motors */
        public DcMotor.ZeroPowerBehavior zeroPowerBehavior; /** Applies to all motors */
    }

    /** Normalize a DTS before passing it in for desirable behavior. */
    void applyDTS(DTS dts);
    void setMode(DcMotor.RunMode runMode);
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior);

}
