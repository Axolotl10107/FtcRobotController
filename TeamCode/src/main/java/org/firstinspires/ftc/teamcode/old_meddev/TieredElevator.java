//MediumAutomaton 2023, for FTC22Robot / PowerNap
package org.firstinspires.ftc.teamcode.old_meddev;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TieredElevator implements DcMotor {
    private int TIER_HIGH = 1030;
    private int TIER_MEDIUM = 800;
    private int TIER_LOW = 500;
    private int TIER_STACKTOP = 230;
    private int TIER_GROUND = 130;

    private double holdPower = 0.2;
    private double movePower = 0.4;
    private int tier = 0;

    private double upperLimit = 1030;
    private double lowerLimit = 0;

    private DcMotor elevatorDrive;
    public ElapsedTime movementTimer;

//    public ElapsedTime lockoutTimer;

    public TieredElevator(DcMotor motor) {
        elevatorDrive = motor;
        movementTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        lockoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        setTieredMode();
    }

    //New Methods
    public void moveToHighTier() {
        setPower(movePower);
        setTargetPosition(TIER_HIGH);
        setPower(holdPower);
        tier = 5;
    }

    public void moveToMediumTier() {
        setPower(movePower);
        setTargetPosition(TIER_MEDIUM);
        setPower(holdPower);
        tier = 4;
    }

    public void moveToLowTier() {
        setPower(movePower);
        setTargetPosition(TIER_LOW);
        setPower(holdPower);
        tier = 3;
    }

    public void moveToStackTier() {
        setPower(movePower);
        setTargetPosition(TIER_STACKTOP);
        setPower(holdPower);
        tier = 2;
    }

    public void moveToGroundTier() {
        setPower(movePower);
        setTargetPosition(TIER_GROUND);
        setPower(holdPower);
        tier = 1;
    }

    public void moveToLowestPosition() {
        setPower(movePower);
        setTargetPosition(0);
        setPower(holdPower);
        tier = 0;
    }

    public void moveToNumberedTier(int tiernum) {
        setPower(movePower);
        switch (tiernum) {//TODO: Use this in a generic tier move function to determine what height to move to, and do power sets around it? May shorten things quite a bit.
            case 1:
                moveToGroundTier();
            case 2:
                moveToStackTier();
            case 3:
                moveToLowTier();
            case 4:
                moveToMediumTier();
            case 5:
                moveToHighTier();
            default:
                setTargetPosition(0);
        }
        setPower(holdPower);
    }

    public void setCurrentTierToCurrentHeight() {
        switch(tier) {
            case 1:
                TIER_GROUND = getCurrentPosition();
            case 2:
                TIER_STACKTOP = getCurrentPosition();
            case 3:
                TIER_LOW = getCurrentPosition();
            case 4:
                TIER_MEDIUM = getCurrentPosition();
            case 5:
                TIER_HIGH = getCurrentPosition();
        }
    }

    public void moveUpOneTier() {
        tier += 1;
        moveToNumberedTier(tier);
    }

    public void moveDownOneTier() {
        tier -= 1;
        moveToNumberedTier(tier);
    }



    public void checkLimits() {
        if (!(lowerLimit < getCurrentPosition() && getCurrentPosition() < upperLimit)) {
            setPower(0);
        }
        if (movementTimer.milliseconds() < 500 && getTargetPosition()-getCurrentPosition() < 150) {
            holdPosition();
            setPower(0);
            setMode(RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setManualMode() {
//        setPower(0);
        setMode(RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTieredMode() {
        holdPosition();
        setMode(RunMode.RUN_TO_POSITION);
    }

    public void holdPosition() {
        setTargetPosition(elevatorDrive.getCurrentPosition());
        setPower(holdPower);
    }

    public void setHoldPower(double power) {
        holdPower = power;
    }

    public double getHoldPower() {
        return holdPower;
    }

    public void setMovePower(double power) {
        movePower = power;
    }

    public double getMovePower() {
        return movePower;
    }



    public void setCurrentPositionAsZero() {
        setTargetPosition(0);
        setMode(RunMode.STOP_AND_RESET_ENCODER);
    }


    //DcMotor Method Overrides
    @Override
    public MotorConfigurationType getMotorType() {
        return elevatorDrive.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        elevatorDrive.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return elevatorDrive.getController();
    }

    @Override
    public int getPortNumber() {
        return elevatorDrive.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        elevatorDrive.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return elevatorDrive.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        elevatorDrive.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return elevatorDrive.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        elevatorDrive.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return elevatorDrive.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return elevatorDrive.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return elevatorDrive.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        elevatorDrive.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return elevatorDrive.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        elevatorDrive.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return elevatorDrive.getDirection();
    }

    @Override
    public void setPower(double power) {
        if ((power < 0 && getCurrentPosition() > lowerLimit) || (power > 0 && getCurrentPosition() < upperLimit)) {
            elevatorDrive.setPower(power);
        } else {
            elevatorDrive.setPower(0);
        }
    }

    @Override
    public double getPower() {
        return elevatorDrive.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return elevatorDrive.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return elevatorDrive.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return elevatorDrive.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return elevatorDrive.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        elevatorDrive.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        elevatorDrive.close();
    }
}
