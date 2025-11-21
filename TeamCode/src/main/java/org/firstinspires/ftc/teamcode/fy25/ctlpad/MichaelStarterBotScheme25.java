package org.firstinspires.ftc.teamcode.fy25.ctlpad;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes.ExponentialAxis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes.LinearAxis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes.MergedAxis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.AxisAsButton;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.MomentaryButton;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.LauncherGate;

/** A controller scheme for driving with independent drive, turn, and strafe axes.
 * Matches the "Dual25" diagram. */
public class MichaelStarterBotScheme25 implements StarterBotScheme25 {

    private final StarterBotState25 state;

    private final Button intakeInButton;
    private final Button intakeOutButton;

    private final Button launcherGateIn;
    private final Button launcherWheelSpinUp;
    private final Button launcherWheelSpinDown;

    private final Button denyEntry;

    private final Button driveSpeedUpButton;
    private final Button driveSpeedDownButton;
    private final Button squareUpButton;

    private final Button distanceUpButton;
    private final Button distanceDownButton;
    private final Button distanceZeroButton;

    private final Axis drive;
    private final Axis turn;
    private final Axis strafe;

    public MichaelStarterBotScheme25(Gamepad driver, Gamepad manipulator) {
        state = new StarterBotState25();

        intakeInButton = new MomentaryButton( () -> manipulator.a );
        intakeOutButton = new MomentaryButton( () -> manipulator.b );

        launcherGateIn = new MomentaryButton( () -> manipulator.left_bumper );
        launcherWheelSpinUp = new AxisAsButton( () -> manipulator.right_trigger, 0.3 );
        launcherWheelSpinDown = new AxisAsButton( () -> manipulator.left_trigger, 0.3 );

        denyEntry = new MomentaryButton( () -> manipulator.x );

        driveSpeedUpButton = new TriggerButton( () -> driver.start );
        driveSpeedDownButton = new TriggerButton( () -> driver.back );
        squareUpButton = new TriggerButton( () -> driver.left_bumper );

        distanceUpButton = new TriggerButton( () -> manipulator.dpad_up );
        distanceDownButton = new TriggerButton( () -> manipulator.dpad_down );
        distanceZeroButton = new TriggerButton( () -> manipulator.dpad_right );

        Axis driveForward = new LinearAxis( () -> driver.right_trigger );
        Axis driveBackward = new LinearAxis( () -> driver.left_trigger );
        drive = new MergedAxis( driveBackward, driveForward );
        turn = new ExponentialAxis( () -> -driver.left_stick_x, 2 ); // positive turn is counterclockwise
        strafe = new LinearAxis( () -> driver.right_stick_x );
    }

    private void updateMovementState() {
        state.setDts(
                new DTS(
                        drive.value(),
                        turn.value(),
                        strafe.value()
                )
        );
    }

    private void updateIntakeState() {
        if ( intakeInButton.isActive() ) {
            state.setIntakeState( RotaryIntake.State.RUNIN );
        } else if ( intakeOutButton.isActive() ) {
            state.setIntakeState( RotaryIntake.State.RUNOUT );
        } else {
            state.setIntakeState( RotaryIntake.State.NONE );
        }
    }

    private void updateLauncherWheelState() {
        if ( launcherWheelSpinUp.isActive() ) {
            state.setRunLaunchWheel( true );
        } else if ( launcherWheelSpinDown.isActive() ) {
            state.setRunLaunchWheel( false );
            state.setAllowEntry( denyEntry.isActive() );
        }
    }

    private void updateLauncherGateState() {
        if ( launcherGateIn.isActive() ) {
            state.setLauncherGateState( LauncherGate.State.OPEN );
        } else {
            state.setLauncherGateState( LauncherGate.State.CLOSED );
        }
    }

    private void updateDriveSpeedUpState() {
        if ( driveSpeedUpButton.isActive() ) {
            state.setMaxDriveSpeed( state.getMaxDriveSpeed() + 0.1 );
        }
    }

    private void updateDriveSpeedDownState() {
        if ( driveSpeedDownButton.isActive() ) {
            state.setMaxDriveSpeed( state.getMaxDriveSpeed() - 0.1 );
        }
    }

    private void updateSquareUpState() {
        state.setSquareUp( squareUpButton.isActive() );
    }

    private void updateDistanceState() {
        if ( distanceUpButton.isActive() ) {
            state.incrementDistance( 1 );
        } else if ( distanceDownButton.isActive() ) {
            state.incrementDistance( -1 );
        } else if ( distanceZeroButton.isActive() ) {
            state.zeroDistance();
        }
    }

    @Override
    public StarterBotState25 getState() {
        updateMovementState();
        updateIntakeState();
        updateDriveSpeedUpState();
        updateDriveSpeedDownState();
        updateSquareUpState();
        updateLauncherGateState();
        updateLauncherWheelState();
        updateDistanceState();

        return state;
    }

}
