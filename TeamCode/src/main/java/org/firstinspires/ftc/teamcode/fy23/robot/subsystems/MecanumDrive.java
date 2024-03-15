package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.fy23.units.DTS;

public interface MecanumDrive {

    class Parameters {
        public boolean present; /** Is the subsystem present on this robot? */

        /** max. individual motor acceleration, in power per second
         * (power loosely represents velocity) */
        public double maxMotorAccel;

        /** prevents jerking - in power */
        public double maxDeltaVEachLoop;

        /** The name of the motor on the left front corner of the drivebase */
        public String leftFrontName;

        /** Direction motor spins when positive power is applied - to drive the motor "backwards",
         * do not set this to reverse! Set the power to a negative value. */
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

    void applyDTS(DTS dts);
    void setMode(DcMotor.RunMode runMode);
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior);

}
