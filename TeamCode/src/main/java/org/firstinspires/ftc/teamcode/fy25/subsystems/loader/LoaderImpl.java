package org.firstinspires.ftc.teamcode.fy25.subsystems.loader;

import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;
import com.qualcomm.robotcore.hardware.Servo;

public class LoaderImpl implements Loader{
    Servo servo;

    public LoaderImpl(Parameters parameters) {

        servo = parameters.device;
        servo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void load() {
        servo.setPosition(0.35);
    }

    @Override
    public void pass() {
        servo.setPosition(0.9);
    }

    @Override
    public boolean isLoad() {
        return servo.getPosition() < 0.2;
    }

    @Override
    public void update() {
    }
}
