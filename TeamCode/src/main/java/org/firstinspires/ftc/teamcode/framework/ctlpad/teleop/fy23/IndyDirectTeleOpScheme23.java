package org.firstinspires.ftc.teamcode.framework.ctlpad.teleop.fy23;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Axis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.Button;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.axes.ButtonAsAxis;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.framework.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.framework.units.DTS;

/** A controller scheme for driving with independent drive, turn, and strafe axes. Matches the "Dual23" diagram. This
 * one does not return a TeleOpState to your OpMode, opting to call subsystem methods directly. The OpMode needs only to
 * call the update() method each loop. This is a proof-of-concept, and the non-direct schemes are recommended for better
 * readibility and configurability of the OpMode. */
public class IndyDirectTeleOpScheme23 {

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
    private double driveSpeed;

    public IndyDirectTeleOpScheme23(Gamepad driver, Gamepad manipulator) {
        this.driver = driver;
        this.manipulator = manipulator;

        state = new TeleOpState23();

        clawOpenButton = new TriggerButton( () -> manipulator.x );
        clawCloseButton = new TriggerButton( () -> manipulator.a );
        planeLaunchButton = new TriggerButton( () -> driver.right_bumper );
        driveSpeedUpButton = new TriggerButton( () -> driver.start );
        driveSpeedDownButton = new TriggerButton( () -> driver.back );
        squareUpButton = new TriggerButton( () -> driver.left_bumper );
        armSlowUp = new ButtonAsAxis( () -> manipulator.dpad_up );
        armSlowDown = new ButtonAsAxis( () -> manipulator.dpad_down );
        armMediumUp = new ButtonAsAxis( () -> manipulator.dpad_right );
        armMediumDown = new ButtonAsAxis( () -> manipulator.dpad_left );
    }

    private void updateMovementState(Robot24 robot) {
        double drive = Range.clip(driver.right_trigger - driver.left_trigger, -driveSpeed, driveSpeed);
        double turn = Range.clip(-driver.left_stick_x, -driveSpeed, driveSpeed); // positive turn is counterclockwise
        double strafe = Range.clip(driver.right_stick_x, -driveSpeed, driveSpeed);
        robot.drive.applyDTS(new DTS(drive, turn, strafe));
    }

    private void updateArmFastMovementState(Robot24 robot) {
        if (!armMovementSet) {
            // if all three run, then the last one is the only one that sets it - the other two have no effect
            robot.arm.setPivotPower(-manipulator.left_stick_y); // Y-Axis is negated (nobody knows why, don't ask)
        }
        if (Math.abs(state.getArmMovement()) > 0.05) { // if we set something meaningful...
            armMovementSet = true;
        }
    }

    private void updateArmMediumMovementState(Robot24 robot) {
        if (!armMovementSet) {
            robot.arm.setPivotPower((armMediumUp.value() - armMediumDown.value()) * 0.2);
        }
        if (Math.abs(state.getArmMovement()) > 0.05) {
            armMovementSet = true;
        }
    }

    private void updateArmSlowMovementState(Robot24 robot) {
        if (!armMovementSet) {
            robot.arm.setPivotPower((armSlowUp.value() - armSlowDown.value()) * 0.15);
        }
        if (Math.abs(state.getArmMovement()) > 0.05) {
            armMovementSet = true;
        }
    }

    private void updateElevatorMovementState(Robot24 robot) {
        robot.arm.setElevatorPower(manipulator.right_trigger - manipulator.left_trigger);
    }

    private void updateClawState(Robot24 robot) {
        if (clawOpenButton.isActive()) {
            robot.claw.setState(Claw.State.OPEN);
        } else if (clawCloseButton.isActive()) {
            robot.claw.setState(Claw.State.CLOSED);
        }
    }

//    private void updateLaunchPlaneState(Robot robot) {
//        if (planeLaunchButton.isActive()) {
//            robot.planeLauncher.launch();
//        }
//    }

    private void updateDriveSpeedUpState(Robot24 robot) {
        if (driveSpeedUpButton.isActive() && driveSpeed < 1) {
            driveSpeed += 0.1;
        }
    }

    private void updateDriveSpeedDownState(Robot24 robot) {
        if (driveSpeedDownButton.isActive() && driveSpeed > 0) {
            driveSpeed -= 0.1;
        }
    }

    private void updateSquareUpState(IMUCorrector imuCorrector) {
        if (squareUpButton.isActive()) {
            imuCorrector.squareUp();
        }
    }

    public void update(Robot24 robot, IMUCorrector imuCorrector) {
        armMovementSet = false;
        updateMovementState(robot);
        updateArmMediumMovementState(robot);
        updateArmSlowMovementState(robot);
        updateArmFastMovementState(robot);
        updateElevatorMovementState(robot);
        updateClawState(robot);
//        updateLaunchPlaneState(robot);
        updateDriveSpeedUpState(robot);
        updateDriveSpeedDownState(robot);
        updateSquareUpState(imuCorrector);
    }

}
