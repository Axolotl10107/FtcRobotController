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
        /** Create a Parameters object and provide parameters that don't have default values.
         * @param present Is this subsystem installed on this robot?
         * @param logoFacingDirection The direction that the REV logo on your Control Hub faces
         * @param usbFacingDirection The direction that the USB port on your Control Hub faces */
        public Parameters(boolean present, RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection, RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection) {
            this.present = present;
            this.logoFacingDirection = logoFacingDirection;
            this.usbFacingDirection = usbFacingDirection;
        }

        /** You already set this in the constructor and cannot set it again. */
        public final boolean present;

        /** You already set this in the constructor and cannot set it again. */
        public final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;

        /** You already set this in the constructor and cannot set it again. */
        public final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;
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
