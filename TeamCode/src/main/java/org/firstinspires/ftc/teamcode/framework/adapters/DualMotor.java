package org.firstinspires.ftc.teamcode.framework.adapters;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Turns two motors into a family, a body that moves as one.
 * Not recommended for new development. {@link DualDcMotorEx} is preferred. */
@Deprecated
public class DualMotor implements DcMotorEx {

    /*
    * The following implementation is largely ported from DcMotorExImpl. It just
    * talks to 2 motor controllers where the original only talks to 1. Sometimes
    * this bothers to cross-check the motors and sometimes it doesn't. Depends
    * on how annoying it would be to do in each case. Add more cross-checks if
    * you want, but everything gets set together so everything should stay
    * together. In theory.
    * */

    DcMotorEx motor1;
    DcMotorControllerEx controller1;
    int port1;

    DcMotorEx motor2;
    DcMotorControllerEx controller2;
    int port2;

    /** The two motors must match exactly in configuration. This constructor
     * will make sure they're the same.
     * {@param motor1} Master; configuration will remain unchanged
     * {@param motor2} Slave; will be reconfigured to match motor1 if needed */
    public DualMotor(DcMotorEx motor1, DcMotorEx motor2) {
        this(motor1, motor2, true);
    }

    /** If safe is true, motor1's configuration will be copied to motor2. If you
     * need the motors to be different for some reason (dangerous!), set safe to
     * false, and motor2 will be left alone.
     * Note: if they don't match, methods that can only really return a single
     * value (like getManufacturer() or isMotorEnabled()) will return the values
     * for motor1 (including getController()!). Numeric values (except for
     * getPortNumber()) will be averaged between both motors.
     * {@param motor1} Master; configuration will remain unchanged
     * {@param motor2} Slave; will be reconfigured to match motor1 if safe is
     * true */
    public DualMotor(DcMotorEx motor1, DcMotorEx motor2, boolean safe) {
        this.motor1 = motor1;
        controller1 = (DcMotorControllerEx) motor1.getController();
        port1 = motor1.getPortNumber();

        this.motor2 = motor2;
        controller2 = (DcMotorControllerEx) motor2.getController();
        port2 = motor2.getPortNumber();

        if (safe) {
            // Transfer motor1's config. to motor2
            // DcMotorSimple
            motor2.setDirection(motor2.getDirection());
            motor2.setPower(motor1.getPower());

            // DcMotor
            motor2.setMotorType(motor1.getMotorType());
            motor2.setZeroPowerBehavior(motor1.getZeroPowerBehavior());
            if (motor1.getPowerFloat()) {
                motor2.setPowerFloat();
            }
            motor2.setTargetPosition(motor1.getTargetPosition());
            motor2.setMode(motor1.getMode());

            // DcMotorEx
            if (motor1.isMotorEnabled()) {
                motor2.setMotorEnable();
            } else {
                motor2.setMotorDisable();
            }
            motor2.setVelocity(motor1.getVelocity());

            motor2.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, motor1.getPIDFCoefficients(RunMode.RUN_USING_ENCODER));
            motor2.setPIDFCoefficients(RunMode.RUN_TO_POSITION, motor1.getPIDFCoefficients(RunMode.RUN_TO_POSITION));
            motor2.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, motor1.getPIDFCoefficients(RunMode.RUN_USING_ENCODER));

            motor2.setTargetPositionTolerance(motor1.getTargetPositionTolerance());
            motor2.setCurrentAlert(motor1.getCurrentAlert(CurrentUnit.MILLIAMPS), CurrentUnit.MILLIAMPS);
        }
    }

// ----------------------------- DcMotorEx -------------------------------------

    /**
     * Individually energizes this particular motor.
     *
     * @see #setMotorDisable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorEnable() {
        motor1.setMotorEnable();
        motor2.setMotorEnable();
    }

    /**
     * Individually de-energizes this particular motor.
     *
     * @see #setMotorEnable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorDisable() {
        motor1.setMotorDisable();
        motor2.setMotorDisable();
    }

    /**
     * Returns whether this motor is energized.
     *
     * @see #setMotorEnable()
     * @see #setMotorDisable()
     */
    @Override
    public boolean isMotorEnabled() {
        return motor1.isMotorEnabled();
    }

    /**
     * Sets the velocity of the motor.
     *
     * @param angularRate the desired ticks per second
     */
    @Override
    public void setVelocity(double angularRate) {
        motor1.setVelocity(angularRate);
        motor2.setVelocity(angularRate);
    }

    /**
     * Sets the velocity of the motor.
     *
     * @param angularRate the desired angular rate, in units per second
     * @param unit        the units in which angularRate is expressed
     * @see #getVelocity(AngleUnit)
     */
    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        motor1.setVelocity(angularRate, unit);
        motor2.setVelocity(angularRate, unit);
    }

    /**
     * Returns the average velocity of the two motors, in ticks per second.
     *
     * @return the current velocity of the motor
     */
    @Override
    public double getVelocity() {
        return ( motor1.getVelocity() + motor2.getVelocity() ) / 2.0;
    }

    /**
     * Returns the average velocity of each motor, in angular units per second.
     *
     * @param unit the units in which the angular rate is desired
     * @return the current velocity of the motor
     * @see #setVelocity(double, AngleUnit)
     */
    @Override
    public double getVelocity(AngleUnit unit) {
        return ( motor1.getVelocity(unit) + motor2.getVelocity(unit) ) / 2.0;
    }

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * Note that in some controller implementations, setting the PID coefficients for one
     * mode on a motor might affect other modes on that motor, or might affect the PID
     * coefficients used by other motors on the same controller (this is not true on the
     * REV Expansion Hub).
     *
     * @param mode            either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     * @see #getPIDCoefficients(RunMode)
     * @deprecated Use {@link #setPIDFCoefficients(RunMode, PIDFCoefficients)} instead
     */
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        motor1.setPIDCoefficients(mode, pidCoefficients);
        motor2.setPIDCoefficients(mode, pidCoefficients);
    }

    /**
     * {@link #setPIDFCoefficients} is a superset enhancement to {@link #setPIDCoefficients}. In addition
     * to the proportional, integral, and derivative coefficients previously supported, a feed-forward
     * coefficient may also be specified. Further, a selection of motor control algorithms is offered:
     * the originally-shipped Legacy PID algorithm, and a PIDF algorithm which avails itself of the
     * feed-forward coefficient. Note that the feed-forward coefficient is not used by the Legacy PID
     * algorithm; thus, the feed-forward coefficient must be indicated as zero if the Legacy PID
     * algorithm is used. Also: the internal implementation of these algorithms may be different: it
     * is not the case that the use of PIDF with the F term as zero necessarily exhibits exactly the
     * same behavior as the use of the LegacyPID algorithm, though in practice they will be quite close.
     * <p>
     * Readers are reminded that DcMotor.RunMode.RUN_TO_POSITION mode makes use of <em>both</em>
     * the coefficients set for RUN_TO_POSITION <em>and</em> the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @param mode
     * @param pidfCoefficients
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPositionPIDFCoefficients(double)
     * @see #getPIDFCoefficients(RunMode)
     */
    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        motor1.setPIDFCoefficients(mode, pidfCoefficients);
        motor2.setPIDFCoefficients(mode, pidfCoefficients);
    }

    /**
     * A shorthand for setting the PIDF coefficients for the DcMotor.RunMode.RUN_USING_ENCODER
     * mode. MotorControlAlgorithm.PIDF is used.
     *
     * @param p
     * @param i
     * @param d
     * @param f
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF));
    }

    /**
     * A shorthand for setting the PIDF coefficients for the DcMotor.RunMode#RUN_TO_POSITION
     * mode. MotorControlAlgorithm.PIDF is used.
     * <p>
     * Readers are reminded that DcMotor.RunMode.RUN_TO_POSITION mode makes use of <em>both</em>
     * the coefficients set for RUN_TO_POSITION <em>and</em> the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @param p
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    @Override
    public void setPositionPIDFCoefficients(double p) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, 0, 0, 0, MotorControlAlgorithm.PIDF));
    }

    /**
     * Returns the PID control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PID control coefficients used when running in the indicated mode on this motor
     * @deprecated Use {@link #getPIDFCoefficients(RunMode)} instead
     */
    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motor1.getPIDCoefficients(mode);
    }

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motor1.getPIDFCoefficients(mode);
    }

    /**
     * Sets the target positioning tolerance of this motor.
     *
     * @param tolerance the desired tolerance, in encoder ticks
     * @see DualMotor#setTargetPosition(int)
     */
    @Override
    public void setTargetPositionTolerance(int tolerance) {
        motor1.setTargetPositionTolerance(tolerance);
        motor2.setTargetPositionTolerance(tolerance);
    }

    /**
     * Returns the current target positioning tolerance of this motor.
     *
     * @return the current target positioning tolerance of this motor
     */
    @Override
    public int getTargetPositionTolerance() {
        return motor1.getTargetPositionTolerance();
    }

    /**
     * Returns the average current consumed by each motor.
     *
     * @param unit current units
     * @return the average current consumed by each motor.
     */
    @Override
    public double getCurrent(CurrentUnit unit) {
        return ( motor1.getCurrent(unit) + motor2.getCurrent(unit) ) / 2.0;
    }

    /**
     * Returns the current alert for this motor.
     *
     * @param unit current units
     * @return the current alert for this motor
     */
    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor1.getCurrentAlert(unit);
    }

    /**
     * Sets the current alert for this motor.
     *
     * @param current current alert
     * @param unit    current units
     */
    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor1.setCurrentAlert(current, unit);
        motor2.setCurrentAlert(current, unit);
    }

    /**
     * Returns whether the current consumption of either motor exceeds the alert threshold.
     *
     * @return whether the current consumption of either motor exceeds the alert threshold.
     */
    @Override
    public boolean isOverCurrent() {
        return ( motor1.isOverCurrent() || motor2.isOverCurrent() );
    }

// ----------------------------- DcMotor ---------------------------------------

    /**
     * Returns the assigned type for this motor. If no particular motor type has been
     * configured, then {@link MotorConfigurationType#getUnspecifiedMotorType()} will be returned.
     * Note that the motor type for a given motor is initially assigned in the robot
     * configuration user interface, though it may subsequently be modified using methods herein.
     *
     * @return the assigned type for this motor
     */
    @Override
    public MotorConfigurationType getMotorType() {
        return motor1.getMotorType();
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     *
     * @param motorType the new assigned type for this motor
     * @see #getMotorType()
     */
    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor1.setMotorType(motorType);
    }

    /**
     * Returns the underlying motor controller on which <b>motor1</b> is situated.
     *
     * @return the underlying motor controller on which <b>motor1</b> is situated.
     * @see #getPortNumber()
     */
    @Override
    public DcMotorController getController() {
        return motor1.getController();
    }

    /**
     * Returns the port number on the underlying motor controller on which <b>motor1</b>> is situated.
     *
     * @return the port number on the underlying motor controller on which this <b>motor1</b>> is situated.
     * @see #getController()
     */
    @Override
    public int getPortNumber() {
        return motor1.getPortNumber();
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     *
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see ZeroPowerBehavior
     * @see #setPower(double)
     */
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor1.setZeroPowerBehavior(zeroPowerBehavior);
        motor2.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Returns the current behavior of <b>motor1</b> were a power level of zero to be applied.
     *
     * @return the current behavior of <b>motor1</b> were a power level of zero to be applied.
     */
    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor1.getZeroPowerBehavior();
    }

    /**
     * Sets the zero power behavior of the motor to {@link ZeroPowerBehavior#FLOAT FLOAT}, then
     * applies zero power to that motor.
     *
     * <p>Note that the change of the zero power behavior to {@link ZeroPowerBehavior#FLOAT FLOAT}
     * remains in effect even following the return of this method. <STRONG>This is a breaking
     * change</STRONG> in behavior from previous releases of the SDK. Consider, for example, the
     * following code sequence:</p>
     *
     * <pre>
     *     motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); // method not available in previous releases
     *     motor.setPowerFloat();
     *     motor.setPower(0.0);
     * </pre>
     *
     * <p>Starting from this release, this sequence of code will leave the motor floating. Previously,
     * the motor would have been left braked.</p>
     *
     * @see #setPower(double)
     * @see #getPowerFloat()
     * @see #setZeroPowerBehavior(ZeroPowerBehavior)
     * @deprecated This method is deprecated in favor of direct use of
     * {@link #setZeroPowerBehavior(ZeroPowerBehavior) setZeroPowerBehavior()} and
     * {@link #setPower(double) setPower()}.
     */
    @Override
    public void setPowerFloat() {
        motor1.setPowerFloat();
        motor2.setPowerFloat();
    }

    /**
     * Returns whether <b>motor1</b> is currently in a float power level.
     *
     * @return whether <b>motor1</b> is currently in a float power level.
     * @see #setPowerFloat()
     */
    @Override
    public boolean getPowerFloat() {
        return motor1.getPowerFloat();
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold thereat. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, {@link #isBusy()} will return true.
     *
     * <p>Note that adjustment to a target position is only effective when the motor is in
     * {@link RunMode#RUN_TO_POSITION RUN_TO_POSITION}
     * RunMode. Note further that, clearly, the motor must be equipped with an encoder in order
     * for this mode to function properly.</p>
     *
     * @param position the desired encoder target position
     * @see #getCurrentPosition()
     * @see #setMode(RunMode)
     * @see RunMode#RUN_TO_POSITION
     * @see #getTargetPosition()
     * @see #isBusy()
     */
    @Override
    public void setTargetPosition(int position) {
        motor1.setTargetPosition(position);
        motor2.setTargetPosition(position);
    }

    /**
     * Returns the current target encoder position for <b>motor1</b>.
     *
     * @return the current target encoder position for <b>motor1</b>.
     * @see #setTargetPosition(int)
     */
    @Override
    public int getTargetPosition() {
        return motor1.getTargetPosition();
    }

    /**
     * Returns true if either motor is currently advancing or retreating to a target position.
     *
     * @return true if either motor is currently advancing or retreating to a target position.
     * @see #setTargetPosition(int)
     */
    @Override
    public boolean isBusy() {
        return ( motor1.isBusy() || motor2.isBusy() );
    }

    /**
     * Returns the average reading of the encoder for each motor. The units for this reading,
     * that is, the number of ticks per revolution, are specific to the motor/encoder in question,
     * and thus are not specified here.
     *
     * @return the average reading of the encoder for each motor
     * @see #getTargetPosition()
     * @see RunMode#STOP_AND_RESET_ENCODER
     */
    @Override
    public int getCurrentPosition() {
        return ( motor1.getCurrentPosition() + motor2.getCurrentPosition() ) / 2;
    }

    /**
     * Sets the current run mode for this motor.
     *
     * @param mode the new current run mode for this motor
     * @see RunMode
     * @see #getMode()
     */
    @Override
    public void setMode(RunMode mode) {
        motor1.setMode(mode);
        motor2.setMode(mode);
    }

    /**
     * Returns the current run mode for <b>motor1</b>.
     *
     * @return the current run mode for <b>motor1</b>
     * @see RunMode
     * @see #setMode(RunMode)
     */
    @Override
    public RunMode getMode() {
        return motor1.getMode();
    }

    /**
     * Sets the logical direction in which this motor operates.
     *
     * @param direction the direction to set for this motor
     * @see #getDirection()
     */
    @Override
    public void setDirection(Direction direction) {
        motor1.setDirection(direction);
        motor2.setDirection(direction);
    }

    /**
     * Returns the current logical direction in which <b>motor1</b> is set as operating.
     *
     * @return the current logical direction in which this <b>motor1</b> is set as operating.
     * @see #setDirection(Direction)
     */
    @Override
    public Direction getDirection() {
        return motor1.getDirection();
    }

    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum
     * possible power / speed supported according to the run mode in which the
     * motor is operating.
     *
     * <p>Setting a power level of zero will brake the motor</p>.
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     * @see #getPower()
     * @see #setMode
     * @see #setPowerFloat()
     */
    @Override
    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    /**
     * Returns the current configured power level of <b>motor1</b>.
     *
     * @return the current level of <b>motor1</b>, a value in the interval [0.0, 1.0]
     * @see #setPower(double)
     */
    @Override
    public double getPower() {
        return motor1.getPower();
    }

// ----------------------------- HardwareDevice --------------------------------

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the manufacturer of <b>motor1</b>
     */
    @Override
    public Manufacturer getManufacturer() {
        return motor1.getManufacturer();
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name for <b>motor1</b>
     */
    @Override
    public String getDeviceName() {
        return motor1.getDeviceName();
    }

    /**
     * Get connection information about this device in a human readable format.
     *
     * @return connection info for <b>motor1</b>
     */
    @Override
    public String getConnectionInfo() {
        return motor1.getConnectionInfo();
    }

    /**
     * Version.
     *
     * @return get the version of <b>motor1</b>
     */
    @Override
    public int getVersion() {
        return motor1.getVersion();
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor1.resetDeviceConfigurationForOpMode();
        motor2.resetDeviceConfigurationForOpMode();
    }

    /**
     * Closes this device.
     */
    @Override
    public void close() {
        motor1.close();
        motor2.close();
    }
}
