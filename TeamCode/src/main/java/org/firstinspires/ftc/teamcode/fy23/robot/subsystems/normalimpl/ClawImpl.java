package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;

/** A normal implementation of {@link Claw}. */
public class ClawImpl implements Claw {

    private Servo servo;
    private double openPosition;
    private double closePosition;
    private Claw.State state = Claw.State.NONE;

    public ClawImpl(Claw.Parameters parameters, HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, parameters.clawServoName);
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
    public void update() {

    }

}
