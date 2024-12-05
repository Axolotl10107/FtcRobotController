package org.firstinspires.ftc.teamcode.fy24.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Axis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Button;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes.ExponentialAxis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes.LinearAxis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes.MergedAxis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes.TwoButtonsAsAxis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.buttons.MomentaryButton;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RotaryIntake;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** A controller scheme for driving with independent drive, turn, and strafe axes.
 * Matches the "Dual24" diagram. */
public class IndyTeleOpScheme24 implements TeleOpScheme24 {

    // Config value: the point on an axis after which the arm will move
    private double armThreshold = 0.05;

    // Config value: the speed (scale of 0 - 1) at which the arm moves with the medium arm buttons
    private double armMediumSpeed = 0.2;

    // Config value: the speed (scale of 0 - 1) at which the arm moves with the slow arm buttons
    private double armSlowSpeed = 0.15;



    private TeleOpState24 state;

    private Button clawOpenButton;
    private Button clawCloseButton;
    private Button intakeInButton;
    private Button intakeOutButton;

    private Button driveSpeedUpButton;
    private Button driveSpeedDownButton;
    private Button squareUpButton;
    private Button brakeButton;

    private Axis armFast;
    private Axis armMedium;
    private Axis armSlow;

    private Axis elevatorIn;
    private Axis elevatorOut;
    private Axis elevator;

    private Axis driveForward;
    private Axis driveBackward;
    private Axis drive;

    private Axis turn;
    private Axis strafe;

    private boolean armMovementSet = false;

    public IndyTeleOpScheme24(Gamepad driver, Gamepad manipulator) {
        state = new TeleOpState24();

        // claw close is analogous to intake in
        clawCloseButton = new TriggerButton( () -> manipulator.a );
        clawOpenButton = new TriggerButton( () -> manipulator.b );
        intakeInButton = new MomentaryButton( () -> manipulator.a );
        intakeOutButton = new MomentaryButton( () -> manipulator.b );

        driveSpeedUpButton = new TriggerButton( () -> driver.start );
        driveSpeedDownButton = new TriggerButton( () -> driver.back );
        squareUpButton = new TriggerButton( () -> driver.left_bumper );
        brakeButton = new TriggerButton( () -> driver.x );

        armFast = new LinearAxis( () -> -manipulator.left_stick_y); // analog stick y-axes need to be negated
        armMedium = new TwoButtonsAsAxis( () -> manipulator.dpad_left, () -> manipulator.dpad_right, armMediumSpeed);
        armSlow = new TwoButtonsAsAxis( () -> manipulator.dpad_down, () -> manipulator.dpad_up, armSlowSpeed );

        elevatorIn = new LinearAxis( () -> manipulator.left_trigger );
        elevatorOut = new LinearAxis( () -> manipulator.right_trigger );
        elevator = new MergedAxis( elevatorIn, elevatorOut );

        driveForward = new LinearAxis( () -> driver.right_trigger );
        driveBackward = new LinearAxis( () -> driver.left_trigger );
        drive = new MergedAxis( driveBackward, driveForward );
        turn = new ExponentialAxis( () -> -driver.left_stick_x, 2); // positive turn is counterclockwise
        strafe = new LinearAxis( () -> driver.right_stick_x);
    }

    private void updateMovementState() {
        state.setDts( new DTS( drive.value(), turn.value(), strafe.value() ) );
    }

    private void applyArmSetpoint( double setpoint ) {
        if ( setpoint > armThreshold ) {
            state.setArmMovement( setpoint );
            armMovementSet = true;
        }
    }

    // Scaling factor of -1 above will negate the left stick
    private void updateArmFastMovementState() {
        armFast.value();
    }

    private void updateArmMediumMovementState() {
        applyArmSetpoint( armMedium.value() );
    }

    private void updateArmSlowMovementState() {
        applyArmSetpoint( armSlow.value() );
    }

    private void updateElevatorMovementState() {
        state.setElevatorMovement( elevator.value() );
    }

    private void updateClawState() {
        if ( clawOpenButton.isActive() ) {
            state.setClawState( Claw.State.OPEN );
        } else if ( clawCloseButton.isActive() ) {
            state.setClawState( Claw.State.CLOSED );
        }
    }

    private void updateIntakeState() {
        if ( intakeInButton.isActive() ) {
            state.setIntakeState( RotaryIntake.State.RUNIN );
        } else if ( intakeOutButton.isActive() ) {
            state.setIntakeState( RotaryIntake.State.RUNOUT );
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

    private void updateBrakeState() {
        state.setBrake( brakeButton.isActive() );
    }

    @Override
    public TeleOpState24 getState() {
        armMovementSet = false;
        updateMovementState();

        // if one runs, don't have the next one just immediately reset the arm power to 0
        updateArmMediumMovementState();
        if (!armMovementSet) {
            updateArmSlowMovementState();
        }
        if (!armMovementSet) {
            updateArmFastMovementState();
        }

        updateElevatorMovementState();
        updateClawState();
        updateIntakeState();
        updateDriveSpeedUpState();
        updateDriveSpeedDownState();
        updateSquareUpState();
        updateBrakeState();

        return state;
    }

}
