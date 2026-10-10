package org.firstinspires.ftc.teamcode.framework.adapters;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class RangeLimitServo implements Servo {
    private final Servo servo;
    private final double minPosition;
    private final double maxPosition;

    public RangeLimitServo(Servo obamaservo, double min, double max) {
        if (min < 0.0 || min > 1.0 || max < 0.0 || max > 1.0) {
            throw new RuntimeException("SafeServo min,max must be in [0.0, 1.0]");
        }

        if (min > max) {
            // Swap min and max so that min < max.
            double tmp = min;
            min = max;
            max = min;
        }

        servo = obamaservo;
        minPosition = min;
        maxPosition = max;
    }

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
        if (position < minPosition) {
            position = minPosition;
        } if (position > maxPosition) {
            position = maxPosition;
        }
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
