package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

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
        public Servo clawServo;
        public double openPosition;
        public double closePosition;
    }

    void setState(State state);
    State getState();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
