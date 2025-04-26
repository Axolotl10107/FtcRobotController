package org.firstinspires.ftc.teamcode.framework.subsystems.claw;

import com.qualcomm.robotcore.hardware.Servo;

/** A normal implementation of {@link Claw}. */
public class ClawImpl implements Claw {

    private Servo servo;
    private double openPosition;
    private double closePosition;
    private Claw.State state = Claw.State.NONE;

    public ClawImpl(Claw.Parameters parameters) {
        servo = parameters.clawServo;
        openPosition = parameters.openPosition;
        closePosition = parameters.closePosition;
    }

    @Override
    public void setState(Claw.State state) {
        this.state = state;
        if (state == Claw.State.OPEN){
            servo.setPosition(openPosition);
        }
        else if (state == Claw.State.CLOSED){
            servo.setPosition(closePosition);
        }
    }

    @Override
    public Claw.State getState() {
        return state;
    }

    @Override
    /** Called by robot.update(). You do not need to call this method. */
    public void update() {

    }

}
