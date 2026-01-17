package org.firstinspires.ftc.teamcode.fy25.subsystems.launchergateservo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.LauncherGate;

public class LauncherGateServoImpl implements  LauncherGateServo {
    Servo servo;
    public LauncherGateServoImpl(Parameters parameters) {
        servo = parameters.device;
    }

    @Override
    public void open() {
        servo.setPosition(-0.2);
    }

    @Override
    public void close() {
        servo.setPosition(0);
    }

    @Override
    public boolean isOpen() {
        return servo.getPosition() > 0.15;
    }

    @Override
    public void update() {

    }
}
