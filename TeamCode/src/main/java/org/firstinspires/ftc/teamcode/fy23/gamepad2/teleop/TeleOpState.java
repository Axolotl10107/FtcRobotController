package org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

public class TeleOpState {

    private DTS dts = new DTS();
    private double armMovement = 0;
    private double elevatorMovement = 0;
    private Claw.State clawState = Claw.State.NONE;
    private boolean launchPlane = false;
    private boolean driveSpeedUp = false;
    private boolean driveSpeedDown = false;

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

    public boolean isDriveSpeedUp() {
        return driveSpeedUp;
    }

    public void setDriveSpeedUp(boolean driveSpeedUp) {
        this.driveSpeedUp = driveSpeedUp;
    }

    public boolean isDriveSpeedDown() {
        return driveSpeedDown;
    }

    public void setDriveSpeedDown(boolean driveSpeedDown) {
        this.driveSpeedDown = driveSpeedDown;
    }
}
