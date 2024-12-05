package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RotaryIntake;

public class RotaryIntakeImpl implements RotaryIntake {
    private double servoPower;
    private CRServo servo;
    private RotaryIntake.State state = RotaryIntake.State.NONE;

    public RotaryIntakeImpl(RotaryIntake.Parameters parameters) {
        servo = parameters.intakeServo;
        servoPower = parameters.servoPower;
    }

    @Override
    public void setState(State state) {
        this.state = state;

        switch (state) {
            case RUNIN:
                servo.setPower(servoPower);
                break;
            case RUNOUT:
                servo.setPower(-servoPower);
                break;
            case STOPPED:
            default:
                servo.setPower(0);
                break;
        }
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public void setPower(double power) {
        servoPower = power;
    }

    @Override
    public double getPower() {
        return servoPower;
    }

    @Override
    public void update() {

    }
}
