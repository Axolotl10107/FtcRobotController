package org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class BlankServo extends BlankHardwareDevice implements Servo {
    @Override
    public ServoController getController() {
        return new BlankServoController();
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return Direction.FORWARD;
    }

    @Override
    public void setPosition(double v) {

    }

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public void scaleRange(double v, double v1) {

    }
}
