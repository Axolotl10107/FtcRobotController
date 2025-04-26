package org.firstinspires.ftc.teamcode.framework.gamepad2.teleop.fy23;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.Button;
import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.axes.ButtonAsAxis;
import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.buttons.MomentaryButton;
import org.firstinspires.ftc.teamcode.framework.gamepad2.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.framework.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMU;
import org.firstinspires.ftc.teamcode.framework.units.DTS;

/** A controller scheme for field-oriented driving. Matches the "FieldyGamepadLS" diagram. */
public class FieldyTeleOpScheme23 implements TeleOpScheme23 {

    private Gamepad driver;
    private Gamepad manipulator;

    private TeleOpState23 state;

    private Button clawOpenButton;
    private Button clawCloseButton;
    private Button planeLaunchButton;
    private Button driveSpeedUpButton;
    private Button driveSpeedDownButton;
    private Button squareUpButton;

    private Axis armSlowUp;
    private Axis armSlowDown;
    private Axis armMediumUp;
    private Axis armMediumDown;

    private boolean armMovementSet = false;

    private FriendlyIMU imu;

    public FieldyTeleOpScheme23(Gamepad driver, Gamepad manipulator, FriendlyIMU imu) {
        this.driver = driver;
        this.manipulator = manipulator;
        this.imu = imu;

        state = new TeleOpState23();

        clawOpenButton = new TriggerButton( () -> manipulator.x );
        clawCloseButton = new TriggerButton( () -> manipulator.a );
        planeLaunchButton = new MomentaryButton( () -> driver.right_bumper );
        driveSpeedUpButton = new TriggerButton( () -> driver.start );
        driveSpeedDownButton = new TriggerButton( () -> driver.back );
        squareUpButton = new TriggerButton( () -> driver.left_bumper );
        armSlowUp = new ButtonAsAxis( () -> manipulator.dpad_up );
        armSlowDown = new ButtonAsAxis( () -> manipulator.dpad_down );
        armMediumUp = new ButtonAsAxis( () -> manipulator.dpad_right );
        armMediumDown = new ButtonAsAxis( () -> manipulator.dpad_left );
    }

    private void updateMovementState(double heading) {
        double drive = driver.right_trigger - driver.left_trigger;
        double turn = -driver.left_stick_x; // positive turn is counterclockwise
        double strafe = driver.right_stick_x;
        state.setDts(new DTS(drive, turn, strafe).rotate(heading));
    }

    private void updateArmFastMovementState() {
//        if (!armMovementSet) {
            // if all three run, then the last one is the only one that sets it - the other two have no effect
            state.setArmMovement(-manipulator.left_stick_y); // Y-Axis is negated (nobody knows why, don't ask)
//        }
//        if (Math.abs(state.getArmMovement()) > 0.05) { // if we set something meaningful...
//            armMovementSet = true;
//        }
    }

    private void updateArmMediumMovementState() {
        if (!armMovementSet) {
            state.setArmMovement((armMediumUp.value() - armMediumDown.value()) * 0.2);
        }
        if (Math.abs(state.getArmMovement()) > 0.05) {
            armMovementSet = true;
        }
    }

    private void updateArmSlowMovementState() {
        if (!armMovementSet) {
            state.setArmMovement((armSlowUp.value() - armSlowDown.value()) * 0.15);
        }
        if (Math.abs(state.getArmMovement()) > 0.05) {
            armMovementSet = true;
        }
    }

    private void updateElevatorMovementState() {
        state.setElevatorMovement(manipulator.right_trigger - manipulator.left_trigger);
    }

    private void updateClawState() {
        if (clawOpenButton.isActive()) {
            state.setClawState(Claw.State.OPEN);
        } else if (clawCloseButton.isActive()) {
            state.setClawState(Claw.State.CLOSED);
        }
    }

    private void updateLaunchPlaneState() {
        if (planeLaunchButton.isActive()) {
            state.setLaunchPlane(true);
            // This state must be held to launch the plane, so I'm not using a TriggerButton here.
        } else {
            state.setLaunchPlane(false);
        }
    }

    private void updateDriveSpeedUpState() {
        if (driveSpeedUpButton.isActive()) {
            state.setMaxDriveSpeed( state.getMaxDriveSpeed() + 0.1 );
        }
    }

    private void updateDriveSpeedDownState() {
        if (driveSpeedDownButton.isActive()) {
            state.setMaxDriveSpeed( state.getMaxDriveSpeed() - 0.1 );
        }
    }

    private void updateSquareUpState() {
        state.setSquareUp(squareUpButton.isActive());
    }

    @Override
    public TeleOpState23 getState() {
        armMovementSet = false;
        updateMovementState(imu.yaw(AngleUnit.RADIANS));
//        updateArmMediumMovementState();
//        updateArmSlowMovementState();
        updateArmFastMovementState();
        updateElevatorMovementState();
        updateClawState();
        updateLaunchPlaneState();
        updateDriveSpeedUpState();
        updateDriveSpeedDownState();
        updateSquareUpState();

        return state;
    }
}
