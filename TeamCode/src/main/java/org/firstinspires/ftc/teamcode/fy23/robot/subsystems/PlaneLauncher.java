package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

/** Represents the plane launcher. */
public interface PlaneLauncher {

    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;
        public Servo planeServo;
    }

    void launch();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
