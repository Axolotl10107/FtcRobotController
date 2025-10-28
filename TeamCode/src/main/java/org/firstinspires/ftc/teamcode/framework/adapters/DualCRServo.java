package org.firstinspires.ftc.teamcode.framework.adapters;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

public class DualCRServo implements CRServo {

    private final CRServo primary;
    private final CRServo secondary;

    /**
     * Create a DualServo to represent 2 physical Servos.
     * @param primary The primary servo. Values will be reported from this servo's perspective.
     * @param secondary The secondary servo.
     */
    public DualCRServo(CRServo primary, CRServo secondary) {
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

    /** Returns -1 */
    @Override
    public int getPortNumber() {
        return -1;
    }

    /** If the direction to set is different from the primary's current direction, this sets both
     * servos to the opposite of their current directions. */
    @Override
    public void setDirection(CRServo.Direction direction) {
        if (direction != primary.getDirection()) {
            if (primary.getDirection() == CRServo.Direction.FORWARD) {
                primary.setDirection(CRServo.Direction.REVERSE);
            } else {
                primary.setDirection(CRServo.Direction.FORWARD);
            }
            if (secondary.getDirection() == CRServo.Direction.FORWARD) {
                secondary.setDirection(CRServo.Direction.REVERSE);
            } else {
                secondary.setDirection(CRServo.Direction.FORWARD);
            }
        }
    }

    /** Returns the direction of the primary servo. */
    @Override
    public CRServo.Direction getDirection() {
        return primary.getDirection();
    }

    @Override
    public void setPower(double power) {
        primary.setPower(power);
        secondary.setPower(power);
    }

    /** Returns the average power between both servos. */
    @Override
    public double getPower() {
        double primaryPower = primary.getPower();
        double secondaryPower = secondary.getPower();
        return ( primaryPower + secondaryPower ) / 2;
    }

    @Override
    public Manufacturer getManufacturer() {
        return primary.getManufacturer();
    }

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
