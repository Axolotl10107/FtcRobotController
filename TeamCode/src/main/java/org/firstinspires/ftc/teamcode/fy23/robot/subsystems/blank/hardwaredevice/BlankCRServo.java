package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.hardwaredevice;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

public class BlankCRServo extends BlankHardwareDevice implements CRServo {
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
    public void setPower(double power) {

    }

    @Override
    public double getPower() {
        return 0;
    }
}
