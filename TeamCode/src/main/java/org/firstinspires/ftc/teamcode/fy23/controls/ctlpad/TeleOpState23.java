package org.firstinspires.ftc.teamcode.fy23.controls.ctlpad;

import org.firstinspires.ftc.teamcode.framework.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.framework.units.DTS;

/** Stores the state of the controls. The control scheme updates this, then the OpMode reads it. This effectively maps
 * buttons to actions. */
public class TeleOpState23 {

    private DTS dts = new DTS();
    private double armMovement = 0;
    private double elevatorMovement = 0;
    private Claw.State clawState = Claw.State.NONE;
    private boolean launchPlane = false;

    private double maxDriveSpeed = 1;

    @Deprecated
    private boolean driveSpeedUp = false;
    @Deprecated
    private boolean driveSpeedDown = false;

    private boolean squareUp = false;

    public DTS getDts() {
        return dts;
    }

    public void setDts(DTS dts) {
        this.dts = dts;
    }

    public double getArmMovement() {
        return armMovement;
    }

    public void setArmMovement(double armMovement) {
        this.armMovement = armMovement;
    }

    public double getElevatorMovement() {
        return elevatorMovement;
    }

    public void setElevatorMovement(double elevatorMovement) {
        this.elevatorMovement = elevatorMovement;
    }

    public Claw.State getClawState() {
        return clawState;
    }

    public void setClawState(Claw.State clawState) {
        this.clawState = clawState;
    }

    public boolean isLaunchPlane() {
        return launchPlane;
    }

    public void setLaunchPlane(boolean launchPlane) {
        this.launchPlane = launchPlane;
    }

    public double getMaxDriveSpeed() {
        return maxDriveSpeed;
    }

    public void setMaxDriveSpeed(double newSpeed) {
        maxDriveSpeed = newSpeed;
    }

    @Deprecated
    public boolean isDriveSpeedUp() {
        return driveSpeedUp;
    }

    @Deprecated
    public void setDriveSpeedUp(boolean driveSpeedUp) {
        this.driveSpeedUp = driveSpeedUp;
    }

    @Deprecated
    public boolean isDriveSpeedDown() {
        return driveSpeedDown;
    }

    @Deprecated
    public void setDriveSpeedDown(boolean driveSpeedDown) {
        this.driveSpeedDown = driveSpeedDown;
    }

    public boolean isSquareUp() {
        return squareUp;
    }

    public void setSquareUp(boolean squareUp) {
        this.squareUp = squareUp;
    }
}
