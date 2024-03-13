package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

public interface Claw {

    enum State {
        OPEN,
        CLOSED,
        NONE
    }

    class Parameters {
        public boolean present;
        public double openPosition;
        public double closePosition;
    }

    void setState(State state);
    State getState();

}
