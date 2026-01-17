package org.firstinspires.ftc.teamcode.fy25.ctlpad;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergateservo.LauncherGateServo;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorintake.MotorIntake;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.LauncherGate;
import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.Indexer;

/** Stores the state of the controls. The control scheme updates this, then the OpMode reads it.
 * This effectively maps buttons to actions. This edition is for 2025-26 Decode. */
public class StarterBotState25 {

    private DTS dts = new DTS();
    private RotaryIntake.State intakeState = RotaryIntake.State.NONE;
    private MotorIntake.State motorIntakeState = MotorIntake.State.NONE;
    //    private LauncherWheel.State launcherWheelState = LauncherWheel.State.STOPPED;
    private boolean runLaunchWheel;
    private LauncherGate.State launcherGateState = LauncherGate.State.CLOSED;

    private LauncherGateServo.State launcherGateServoState = LauncherGateServo.State.CLOSED;
    private Indexer.State indexerState = Indexer.State.READY;
    private Indexer.Index index = Indexer.Index.A;

    private Indexer.Index indexGoal = Indexer.Index.A;
    private double maxDriveSpeed = 1;
    private double launchVel = 168000.0;

    public double getIntakeVel() {
        return intakeVel;
    }

    public void setIntakeVel(double intakeVel) {
        this.intakeVel = intakeVel;
    }

    public double getLaunchVel() {
        return launchVel;
    }

    public void setLaunchVel(double launchVel) {
        this.launchVel = launchVel;
    }

    private double intakeVel = 2000.0;

    private boolean squareUp = false;
    private boolean brake = false;
    private boolean isAllow = false;

    private double distance = 0;

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

    public void setIndexState(Indexer.State state) {indexerState = state;}

    public Indexer.State getIndexState() {return indexerState;}

    public void setIndex(Indexer.Index index) {this.index = index;}

    public Indexer.Index getIndex() {return index;}

    public void setIndexGoal(Indexer.Index index) {indexGoal = index;}

    public Indexer.Index getIndexGoal() {return indexGoal;}

    public MotorIntake.State getMotorIntakeState() {return motorIntakeState;}

    public void setMotorIntakeState(MotorIntake.State state) {motorIntakeState = state;}

    //TODO add loader handling

    public void setRunLaunchWheel(boolean runLaunchWheel) {
        this.runLaunchWheel = runLaunchWheel;
    }

    public boolean isRunLaunchWheel() {
        return runLaunchWheel;
    }

    public void setLauncherGateState(LauncherGate.State state) {launcherGateState = state;}

    public void setLauncherGateServoState(LauncherGateServo.State state) {launcherGateServoState = state;}

    public LauncherGate.State getLauncherGateState() {return launcherGateState;}
    public LauncherGateServo.State getLauncherGateServoState() {return launcherGateServoState;}

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

    public void incrementDistance(double value) {
        if (distance + value >= 0) {
            distance += value;
        }
    }

    public void zeroDistance() {
        distance = 0;
    }

    public double getDistance() {return distance;}

    public void setAllowEntry(boolean b) {isAllow = b;}

    public boolean getAllowEntry() {return isAllow;}
}
