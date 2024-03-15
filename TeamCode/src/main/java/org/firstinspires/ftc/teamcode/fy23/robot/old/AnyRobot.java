package org.firstinspires.ftc.teamcode.fy23.robot.old;

import org.firstinspires.ftc.teamcode.fy23.units.PIDconsts;

/** Hopefully allows something to accept any robot, when that makes any sense */
public interface AnyRobot {

    /** Ticks per Rotation - how much the encoder increments on each rotation of a drive motor */
    public final double TPR = 0;

    /** Wheel diameter in centimeters */
    public final double wheelDiameter = 9.6;

    /** Maximum forward speed in centimeters per second */
    public final double maxForwardSpeed = 54;

    /** Default TunablePID tuning, for when using the IMUdrive pipeline */
    // TODO: This needs a better name
    public final PIDconsts pidConsts = new PIDconsts(0,0,0);

    /** Default PID constants for the SDK's PID algorithm on individual DcMotorEx devices. Useful
     * for {@link RudimentaryRampToTarget}, perhaps, which
     * uses DcMotorEx.setVelocity() in the RUN_USING_ENCODER runmode. */
    public final PIDconsts sdkMotorPidConsts = new PIDconsts(0,0,0);
}
