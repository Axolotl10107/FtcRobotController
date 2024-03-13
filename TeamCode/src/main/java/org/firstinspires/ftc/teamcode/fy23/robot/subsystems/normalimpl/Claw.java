package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo servo;
    private double openPosition;
    private double closePosition;
    private State state = State.NONE;

    public enum State {
        OPEN,
        CLOSED,
        NONE
    }

    public static class Parameters {
        public boolean present;
        public double openPosition;
        public double closePosition;
    }

    public Claw(Parameters parameters, HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "clawServo");
        openPosition = parameters.openPosition;
        closePosition = parameters.closePosition;
    }

    public void setState(State state) {
        this.state = state;
        if (state == State.OPEN){
            servo.setPosition(openPosition);
        }
        else if (state == State.CLOSED){
            servo.setPosition(closePosition);
        }
    }

    public State getState() {
        return state;
    }
}
