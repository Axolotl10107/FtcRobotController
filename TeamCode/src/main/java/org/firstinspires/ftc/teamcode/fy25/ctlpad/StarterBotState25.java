package org.firstinspires.ftc.teamcode.fy25.ctlpad;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorlntake.MotorIntake;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.LauncherGate;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheel;

/** Stores the state of the controls. The control scheme updates this, then the OpMode reads it.
 * This effectively maps buttons to actions. This edition is for 2025-26 Decode. */
public class StarterBotState25 {

    private DTS dts = new DTS();
    private RotaryIntake.State intakeState = RotaryIntake.State.NONE;
    private MotorIntake.State motorIntakeState = MotorIntake.State.NONE;
    //    private LauncherWheel.State launcherWheelState = LauncherWheel.State.STOPPED;
    private boolean runLaunchWheel;
    private LauncherGate.State launcherGateState = LauncherGate.State.CLOSED;

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

    public MotorIntake.State getMotorIntakeState() {return motorIntakeState;}

    public void setMotorIntakeState(MotorIntake.State state) {motorIntakeState = state;}

//    public void setLauncherWheelState(LauncherWheel.State state) {launcherWheelState = state;}
//
//    public LauncherWheel.State getLauncherWheelState() {return launcherWheelState;}

    public boolean isRunLaunchWheel() {
        return runLaunchWheel;
    }

    public void setRunLaunchWheel(boolean runLaunchWheel) {
        this.runLaunchWheel = runLaunchWheel;
    }

    public void setLauncherGateState(LauncherGate.State state) {launcherGateState = state;}

    public LauncherGate.State getLauncherGateState() {return launcherGateState;}

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
