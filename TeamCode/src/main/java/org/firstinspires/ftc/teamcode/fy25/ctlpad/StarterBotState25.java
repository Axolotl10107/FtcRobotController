package org.firstinspires.ftc.teamcode.fy25.ctlpad;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.framework.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.DTS;

/** Stores the state of the controls. The control scheme updates this, then the OpMode reads it.
 * This effectively maps buttons to actions. This edition is for 2025-26 Decode. */
public class StarterBotState25 {

    private DTS dts = new DTS();
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

    public RotaryIntake.State getIntakeState() {
        return intakeState;
    }

    public void setIntakeState(RotaryIntake.State state) {
        intakeState = state;
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

    /** Please use a {@link TriggerButton} for this. */
    public void setSquareUp(boolean squareUp) {
        this.squareUp = squareUp;
    }

    /** Please use a {@link TriggerButton} for this. */
    public void setBrake(boolean brake) {
        this.brake = brake;
    }
}
