package org.firstinspires.ftc.teamcode.framework.adapters;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/** Turns two Servos into one. */
public class DualServo implements Servo {

    private final Servo primary;
    private final Servo secondary;

    /**
     * Create a DualServo to represent 2 physical Servos.
     * @param primary The primary servo. Values will be reported from this servo's perspective.
     * @param secondary The secondary servo.
     */
    public DualServo(Servo primary, Servo secondary) {
        this.primary = primary;
        this.secondary = secondary;
    }

    /** Returns null! Don't use this! */
    @Override
    public ServoController getController() {
//        return new BlankServoController();
        // This is to fail fast if someone ends up using this.
        // It's not safe for something to interact more directly with just one servo.
        return null;
    }

    /** Returns -1. */
    @Override
    public int getPortNumber() {
        return -1;
    }

    /** If the direction to set is different from the primary's current direction, this sets both
     * servos to the opposite of their current directions. */
    @Override
    public void setDirection(Direction direction) {
        if (direction != primary.getDirection()) {
            if (primary.getDirection() == Direction.FORWARD) {
                primary.setDirection(Direction.REVERSE);
            } else {
                primary.setDirection(Direction.FORWARD);
            }
            if (secondary.getDirection() == Direction.FORWARD) {
                secondary.setDirection(Direction.REVERSE);
            } else {
                secondary.setDirection(Direction.FORWARD);
            }
        }
    }

    /** Returns the direction of the primary servo. */
    @Override
    public Direction getDirection() {
        return primary.getDirection();
    }

    @Override
    public void setPosition(double position) {
        primary.setPosition(position);
        secondary.setPosition(position);
    }

    /** Returns the average position between both servos. */
    @Override
    public double getPosition() {
        double primaryPos = primary.getPosition();
        double secondaryPos = secondary.getPosition();
        return ( primaryPos + secondaryPos ) / 2;
    }

    @Override
    public void scaleRange(double min, double max) {
        primary.scaleRange(min, max);
        secondary.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return primary.getManufacturer();
    }

    /** Returns "Dual" followed by the primary's device name then the secondary's device name. */
    @Override
    public String getDeviceName() {
        return "Dual " + primary.getDeviceName() + " " + secondary.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return primary.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return primary.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        primary.resetDeviceConfigurationForOpMode();
        secondary.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        primary.close();
        secondary.close();
    }
}
