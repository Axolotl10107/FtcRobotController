package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

/** Represents the plane launcher. */
public interface PlaneLauncher {

    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;
        public String planeServoName;
    }

    void launch();

    void update();

}
