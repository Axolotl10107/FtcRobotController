package org.firstinspires.ftc.teamcode.fy25.ctlpad;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes.ButtonAsAxis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes.ExponentialAxis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes.LinearAxis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes.MergedAxis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.AxisAsButton;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.MomentaryButton;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.LauncherGate;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheel;

/** A controller scheme for driving with independent drive, turn, and strafe axes.
 * Matches the "Dual25" diagram. */
public class IndyStarterBotScheme25 implements StarterBotScheme25 {

    // A few config values up here for easy access.

    // Config value: the point on an axis after which the arm will move
    @SuppressWarnings("FieldCanBeLocal")
    private final double ARM_THRESHOLD = 0.05;

    // Config value: the speed (scale of 0 - 1) at which the arm moves with the medium arm buttons
    @SuppressWarnings("FieldCanBeLocal")
    private final double ARM_MEDIUM_SPEED = 0.2;

    // Config value: the speed (scale of 0 - 1) at which the arm moves with the slow arm buttons
    @SuppressWarnings("FieldCanBeLocal")
    private final double ARM_SLOW_SPEED = 0.15;



    private final StarterBotState25 state;

    private final Button clawOpenButton;
    private final Button clawCloseButton;
    private final Button intakeInButton;
    private final Button intakeOutButton;

    private final Axis launcherGateIn;
    private final Button launcherWheelSpinUp;
//    private final Button launcherWheelSpinDown;

    private final Button driveSpeedUpButton;
    private final Button driveSpeedDownButton;
    private final Button squareUpButton;
    private final Button brakeButton;

    private final Button distanceUpButton;
    private final Button distanceDownButton;
    private final Button distanceZeroButton;

    private final Axis armFast;
    private final Axis armMedium;
    private final Axis armSlow;

    private final Axis elevator;

    private final Axis drive;

    private final Axis turn;
    private final Axis strafe;

    private boolean armMovementSet = false;

    public IndyStarterBotScheme25(Gamepad driver, Gamepad manipulator) {
        state = new StarterBotState25();

        // claw close is analogous to intake in
        clawCloseButton = new TriggerButton( () -> manipulator.a );
        clawOpenButton = new TriggerButton( () -> manipulator.b );
        intakeInButton = new MomentaryButton( () -> manipulator.a );
        intakeOutButton = new MomentaryButton( () -> manipulator.b );

        launcherGateIn = new LinearAxis( () -> manipulator.left_trigger);
//        launcherWheelSpinUp = new LinearAxis( () -> manipulator.right_trigger);
        launcherWheelSpinUp = new AxisAsButton( () -> manipulator.right_trigger, 0.3 );
//        launcherWheelSpinDown = new AxisAsButton( () -> manipulator.left_trigger, 0.3 );

        driveSpeedUpButton = new TriggerButton( () -> driver.start );
        driveSpeedDownButton = new TriggerButton( () -> driver.back );
        squareUpButton = new TriggerButton( () -> driver.left_bumper );
        brakeButton = new TriggerButton( () -> driver.x );

        distanceUpButton = new MomentaryButton(() -> manipulator.dpad_right);
        distanceDownButton = new MomentaryButton(() -> manipulator.dpad_left);
        distanceZeroButton = new MomentaryButton(() -> manipulator.x);

        armFast = new LinearAxis( () -> -manipulator.left_stick_y); // analog stick y-axes need to be negated
        armMedium = new MergedAxis( new ButtonAsAxis( new MomentaryButton( () -> manipulator.dpad_left ), -ARM_MEDIUM_SPEED ), new ButtonAsAxis( new MomentaryButton( () -> manipulator.dpad_right ), ARM_MEDIUM_SPEED ) );
        armSlow = new MergedAxis( new ButtonAsAxis( new MomentaryButton( () -> manipulator.dpad_down ), -ARM_SLOW_SPEED ), new ButtonAsAxis( new MomentaryButton( () -> manipulator.dpad_up ), ARM_SLOW_SPEED ) );

        Axis elevatorIn = new LinearAxis( () -> manipulator.left_trigger );
        Axis elevatorOut = new LinearAxis( () -> manipulator.right_trigger );
        elevator = new MergedAxis( elevatorIn, elevatorOut );

        Axis driveForward = new LinearAxis( () -> driver.right_trigger );
        Axis driveBackward = new LinearAxis( () -> driver.left_trigger );
        drive = new MergedAxis( driveBackward, driveForward );
        turn = new ExponentialAxis( () -> -driver.left_stick_x, 2); // positive turn is counterclockwise
        strafe = new LinearAxis( () -> driver.right_stick_x);
    }

    private void updateMovementState() {
        state.setDts( new DTS( drive.value(), turn.value(), strafe.value() ) );
    }

//    private void applyArmSetpoint( double setpoint ) {
//        if ( setpoint > ARM_THRESHOLD) {
//            state.setArmMovement( setpoint );
//            armMovementSet = true;
//        }
//    }

    // Scaling factor of -1 above will negate the left stick
//    private void updateArmFastMovementState() {
//        armFast.value();
//    }
//
//    private void updateArmMediumMovementState() {
//        applyArmSetpoint( armMedium.value() );
//    }
//
//    private void updateArmSlowMovementState() {
//        applyArmSetpoint( armSlow.value() );
//    }
//
//    private void updateElevatorMovementState() {
//        state.setElevatorMovement( elevator.value() );
//    }

//    private void updateClawState() {
//        if ( clawOpenButton.isActive() ) {
//            state.setClawState( Claw.State.OPEN );
//        } else if ( clawCloseButton.isActive() ) {
//            state.setClawState( Claw.State.CLOSED );
//        }
//    }

    private void updateIntakeState() {
        if ( intakeInButton.isActive() ) {
            state.setIntakeState( RotaryIntake.State.RUNIN );
        } else if ( intakeOutButton.isActive() ) {
            state.setIntakeState( RotaryIntake.State.RUNOUT );
        } else {
            state.setIntakeState(RotaryIntake.State.NONE);
        }
    }

    private void updateLauncherWheelFrontState() {
        state.setRunLaunchWheelFront(launcherWheelSpinUp.isActive());

//        if (launcherWheelSpinUp.isActive()) {
//            state.setRunLaunchWheel(true);
//        }
//        if (launcherWheelSpinDown.isActive()) {
//            state.setRunLaunchWheel(false);
//        }
//        if (launcherWheelSpinUp.value() > 0) {
//            state.setLauncherWheelState(LauncherWheel.State.RUNOUT);
//        } else {
//            state.setLauncherWheelState(LauncherWheel.State.STOPPED);
//        }
    }

    private void updateLauncherWheelBackState() {
        state.setRunLaunchWheelBack(launcherWheelSpinUp.isActive());
    }

    private void updateLauncherGateState() {
        if (launcherGateIn.value() > 0) {
            state.setLauncherGateState(LauncherGate.State.OPEN);
        } else {
            state.setLauncherGateState(LauncherGate.State.CLOSED);
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

    private void updateDistanceState() {
        if (distanceUpButton.isActive()) {
            state.incrementDistance(1);
        } else if (distanceDownButton.isActive()) {
            state.incrementDistance(-1);
        }

        if (distanceZeroButton.isActive()) {
            state.zeroDistance();
        }
    }

    @Override
    public StarterBotState25 getState() {
        armMovementSet = false;
        updateMovementState();

        // if one runs, don't have the next one just immediately reset the arm power to 0
//        updateArmMediumMovementState();
//        if (!armMovementSet) {
//            updateArmSlowMovementState();
//        }
//        if (!armMovementSet) {
//            updateArmFastMovementState();
//        }
//
//        updateElevatorMovementState();
//        updateClawState();
        updateIntakeState();
        updateDriveSpeedUpState();
        updateDriveSpeedDownState();
        updateSquareUpState();
        updateBrakeState();
        updateLauncherGateState();
        updateLauncherWheelFrontState();
        updateLauncherWheelBackState();
        updateDistanceState();

        return state;
    }

}
