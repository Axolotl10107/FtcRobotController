package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Represents the IMU built into the control hub. Currently only supports the BNO055, but there is a task on the board
 * (albeit of very low priority) to use the newer IMU interface instead. Wraps the IMU already available in the SDK,
 * making pitch, roll, and yaw easily accessible as methods rather than the SDK's more complicated ways of obtaining
 * them.
 * AxesOrder is ZYX, because most of our uses for the IMU rely on accurate yaw (Z axis) information.
 * Unless an AngleUnit is specified, returned values will be in degrees. */
public interface FriendlyIMU {

    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;

        /** The direction that the REV logo on your Control Hub faces */
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;

        /** The direction that the USB port on your Control Hub faces */
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;
    }

    /** X rotation */
    double pitch();
    /** X rotation */
    double pitch(AngleUnit angleUnit);

    /** Velocity of X rotation */
    double pitchVel();
    /** Velocity of X rotation */
    double pitchVel(AngleUnit angleUnit);

    /** Y rotation */
    double roll();
    /** Y rotation */
    double roll(AngleUnit angleUnit);

    /** Velocity of Y rotation **/
    double rollVel();
    /** Velocity of Y rotation **/
    double rollVel(AngleUnit angleUnit);

    /** Z rotation */
    double yaw();
    /** Z rotation */
    double yaw(AngleUnit angleUnit);

    /** Velocity of Z rotation */
    double yawVel();
    /** Velocity of Z rotation */
    double yawVel(AngleUnit angleUnit);

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
