package org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Axis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Button;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.axes.ButtonAsAxis;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.buttons.MomentaryButton;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

public class IndyTeleOpScheme {

    private Gamepad driver;
    private Gamepad manipulator;

    private TeleOpState state;

    private Button clawOpenButton;
    private Button clawCloseButton;
    private Button planeLaunchButton;
    private Button driveSpeedUpButton;
    private Button driveSpeedDownButton;

    private Axis armSlowUp;
    private Axis armSlowDown;
    private Axis armMediumUp;
    private Axis armMediumDown;

    IndyTeleOpScheme(Gamepad driver, Gamepad manipulator) {
        this.driver = driver;
        this.manipulator = manipulator;
        clawOpenButton = new TriggerButton( () -> manipulator.x );
        clawCloseButton = new TriggerButton( () -> manipulator.a );
        planeLaunchButton = new MomentaryButton( () -> driver.right_bumper );
        driveSpeedUpButton = new TriggerButton( () -> driver.start );
        driveSpeedDownButton = new TriggerButton( () -> driver.back );
        armSlowUp = new ButtonAsAxis( () -> manipulator.dpad_up );
        armSlowDown = new ButtonAsAxis( () -> manipulator.dpad_down );
        armMediumUp = new ButtonAsAxis( () -> manipulator.dpad_right );
        armMediumDown = new ButtonAsAxis( () -> manipulator.dpad_left );
    }

    private void updateMovementState() {
        double drive = driver.right_trigger - driver.left_trigger;
        double turn = driver.left_stick_x;
        double strafe = driver.right_stick_x;
        state.setDts(new DTS(drive, turn, strafe));
    }

    private void updateArmFastMovementState() {
        state.setArmMovement(manipulator.left_stick_y);
    }

    private void updateArmMediumMovementState() {
        state.setArmMovement((armMediumUp.value() - armMediumDown.value()) * 0.2);
    }

    private void updateArmSlowMovementState() {
        state.setArmMovement((armSlowUp.value() - armSlowDown.value()) * 0.15);
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
        state.setDriveSpeedUp(driveSpeedUpButton.isActive());
    }

    private void updateDriveSpeedDownState() {
        state.setDriveSpeedDown(driveSpeedDownButton.isActive());
    }

    public TeleOpState getState() {
        updateMovementState();
        updateArmMediumMovementState();
        updateArmSlowMovementState();
        updateArmFastMovementState();
        updateElevatorMovementState();
        updateClawState();
        updateLaunchPlaneState();
        updateDriveSpeedUpState();
        updateDriveSpeedDownState();
        return state;
    }

}
