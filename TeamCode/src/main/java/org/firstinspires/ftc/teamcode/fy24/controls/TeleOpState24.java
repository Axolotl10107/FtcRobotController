package org.firstinspires.ftc.teamcode.fy24.controls;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RotaryIntake;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** Stores the state of the controls. The control scheme updates this, then the OpMode reads it.
 * This effectively maps buttons to actions. This edition is for 2024-25 Into the Deep. */
public class TeleOpState24 {

    private DTS dts = new DTS();
    private double armMovement = 0;
    private double elevatorMovement = 0;
    private Claw.State clawState = Claw.State.NONE;
    private RotaryIntake.State intakeState = RotaryIntake.State.NONE;

    private double maxDriveSpeed = 1;

    private boolean squareUp = false;
    private boolean brake = false;

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

    public RotaryIntake.State getIntakeState() {
        return intakeState;
    }

    public void setClawState(Claw.State clawState) {
        this.clawState = clawState;
    }

    public void setIntakeState(RotaryIntake.State state) {
        this.intakeState = intakeState;
    }

    public double getMaxDriveSpeed() {
        return maxDriveSpeed;
    }

    public void setMaxDriveSpeed(double newSpeed) {
        maxDriveSpeed = newSpeed;
    }

    public boolean isSquareUp() {
        return squareUp;
    }

    public boolean isBrake() {
        return brake;
    }

    /** Please use a {@link org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.buttons.TriggerButton} for this. */
    public void setSquareUp(boolean squareUp) {
        this.squareUp = squareUp;
    }

    /** Please use a {@link org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.buttons.TriggerButton} for this. */
    public void setBrake(boolean brake) {
        this.brake = brake;
    }
}
