package org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** A testing OpMode that prints TeleOpState to telemetry. */
@TeleOp
public class TeleOpStatePrinter extends OpMode {

    IndyTeleOpScheme scheme;
    private double driveSpeed;

    @Override
    public void init() {
        scheme = new IndyTeleOpScheme(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        TeleOpState state = scheme.getState();
        telemetry.addData("Drive", state.getDts().drive);
        telemetry.addData("Turn", state.getDts().turn);
        telemetry.addData("Strafe", state.getDts().strafe);
        telemetry.addData("Arm Pivot", state.getArmMovement());
        telemetry.addData("Elevator", state.getElevatorMovement());
        telemetry.addData("Claw State", state.getClawState());
        telemetry.addData("Launch Plane?", state.isLaunchPlane());
//        telemetry.addData("Drive Speed Up?", state.isDriveSpeedUp());
//        telemetry.addData("Drive Speed Down?", state.isDriveSpeedDown());

        if (state.isDriveSpeedUp()) {
            driveSpeed += 1;
        } else if (state.isDriveSpeedDown()) {
            driveSpeed -= 1;
        }

        telemetry.addData("Drive Speed", driveSpeed);
    }
}
