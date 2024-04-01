package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

/** Represents the IMU built into the control hub. Currently only supports the BNO055, but there is a task on the board
 * (albeit of very low priority) to use the newer IMU interface instead. Wraps the IMU already available in the SDK,
 * making pitch, roll, and yaw easily accessible as methods rather than the SDK's more complicated ways of obtaining
 * them.*/
public interface FriendlyIMU {

    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;
    }

    double pitch();
    double roll();
    double yaw();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
