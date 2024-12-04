package org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy24;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Axis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Button;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes.ButtonAsAxis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes.LinearAxis;
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



    private Gamepad driver;
    private Gamepad manipulator;

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

    private Axis armSlowUp;
    private Axis armSlowDown;
    private Axis armMediumUp;
    private Axis armMediumDown;

    private boolean armMovementSet = false;

    public IndyTeleOpScheme24(Gamepad driver, Gamepad manipulator) {
        this.driver = driver;
        this.manipulator = manipulator;

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

        armFast = new LinearAxis( () -> manipulator.left_stick_y, -1);
        armMedium = new TwoButtonsAsAxis( () -> manipulator.dpad_left, () -> manipulator.dpad_right);
        armSlow = new TwoButtonsAsAxis( () -> manipulator.dpad_down, () -> manipulator.dpad_up );

        armSlowUp = new ButtonAsAxis( () -> manipulator.dpad_up );
        armSlowDown = new ButtonAsAxis( () -> manipulator.dpad_down );
        armMediumUp = new ButtonAsAxis( () -> manipulator.dpad_right );
        armMediumDown = new ButtonAsAxis( () -> manipulator.dpad_left );
    }

    private void updateMovementState() {
        double drive = driver.right_trigger - driver.left_trigger;
        double turn = -driver.left_stick_x; // positive turn is counterclockwise
        double strafe = driver.right_stick_x;
        state.setDts( new DTS( drive, turn, strafe ) );
    }

    private void applyArmSetpoint( double setpoint ) {
        if ( setpoint > armThreshold ) {
            state.setArmMovement( setpoint );
            armMovementSet = true;
        }
    }

    private void updateArmFastMovementState() {
        applyArmSetpoint( -manipulator.left_stick_y );
    }

    private void updateArmMediumMovementState() {
        applyArmSetpoint( ( armMediumUp.value() - armMediumDown.value() ) * armMediumSpeed );
    }

    private void updateArmSlowMovementState() {
        applyArmSetpoint( ( armSlowUp.value() - armSlowDown.value() ) * armSlowSpeed );
    }

    private void updateElevatorMovementState() {
        state.setElevatorMovement( manipulator.right_trigger - manipulator.left_trigger );
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
        // if all three run, then the last one is the only one that sets it - the other two have no effect
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
