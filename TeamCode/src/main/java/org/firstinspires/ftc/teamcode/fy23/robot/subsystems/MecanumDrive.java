package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.BlankMotor;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** Represents a "mecanum" drive motor layout. Pass in a DTS with the intended movement of the drivebase, and the
 * implementation will handle mapping that to the individual motors.
 * {@link MecanumDrive} has been superseded by {@link org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive}.
 * Please use that instead. */
public interface MecanumDrive {

    /** Contains motor names and settings - usually part of a set of Robot parameters */
    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;

        /** An AccelLimiter object, already instantiated */
        public AccelLimiter accelLimiter;

        /** The motor object on the left front corner of the drivebase, already grabbed from the HardwareMap and
         * configured (direction set) */
        public DcMotorEx leftFrontMotor;
        /** The motor object on the right front corner of the drivebase, already grabbed from the HardwareMap and
         * configured (direction set) */
        public DcMotorEx rightFrontMotor;
        /** The motor object on the left back corner of the drivebase, already grabbed from the HardwareMap and
         * configured (direction set) */
        public DcMotorEx leftBackMotor;
        /** The motor object on the right back corner of the drivebase, already grabbed from the HardwareMap and
         * configured (direction set) */
        public DcMotorEx rightBackMotor;

        /** Applies to all motors */
        public DcMotor.RunMode runMode;

        /** Applies to all motors */
        public DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    }

    /** The leftFront motor, if direct access is needed */
    DcMotorEx leftFront = new BlankMotor();
    /** The rightFront motor, if direct access is needed */
    DcMotorEx rightFront = new BlankMotor();
    /** The leftBack motor, if direct access is needed */
    DcMotorEx leftBack = new BlankMotor();
    /** The rightBack motor, if direct access is needed */
    DcMotorEx rightBack = new BlankMotor();

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
