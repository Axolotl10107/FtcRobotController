package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

/** Represents a claw and its state (open or closed). */
public interface Claw {

    enum State {
        OPEN,
        CLOSED,
        NONE
    }

    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;
        public String clawServoName;
        public double openPosition;
        public double closePosition;
    }

    void setState(State state);
    State getState();

    void update();

}
