//MediumAutomaton 2023, for FTC22Robot / PowerNap
package org.firstinspires.ftc.teamcode.old_meddev;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoWrap implements Servo {
    public final Servo servo;
    private double lastPos;
    private ElapsedTime timeout;

    public ServoWrap(Servo argservo) {
        servo = argservo;
        lastPos = getPosition();
        timeout = new ElapsedTime();
    }

    //New Methods
    public boolean isBusy() {
        double newPos = servo.getPosition();
        boolean busy = (newPos == lastPos || timeout.milliseconds() > 1500);//1500ms timeout
        lastPos = newPos;
        return !busy;//Condition actually returns true when *not* busy :(
    }

    //Original Servo Interface Overrides
    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        timeout.reset();//Busy fallback timeout is 1500 seconds *after* movement began.
        servo.setPosition(position);
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servo.close();
    }
}
