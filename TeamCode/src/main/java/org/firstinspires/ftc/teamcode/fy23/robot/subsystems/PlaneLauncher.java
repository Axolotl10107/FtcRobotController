package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.hardwaredevice.BlankServo;

/** Represents the plane launcher. */
public interface PlaneLauncher {

    class Parameters {
        /** Create a Parameters object and provide parameters that don't have default values.
         * @param present Is this subsystem installed on the robot?
         * @param releasePosition The position the servo should be at to release the rubberband
         * @param restPosition The position the servo should be at to enable resetting the rubberband
         */
        public Parameters(boolean present, double releasePosition, double restPosition) {
            this.present = present;
            this.releasePosition = releasePosition;
            this.restPosition = restPosition;
        }

        /** You already set this in the constructor and cannot set it again. */
        public final boolean present;

        /** The servo that releases the rubberband, already instantiated and configured */
        public Servo planeServo = new BlankServo();

        /** You already set this in the constructor and cannot set it again. */
        public final double releasePosition;
        /** You already set this in the constructor and cannot set it again. */
        public final double restPosition;
    }

    /** Release the rubberband to launch the plane. (This occurs asynchronously, so you need only call it once. The plane
     * will launch, and the servo will automatically return to the restPosition.) */
    void launch();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
