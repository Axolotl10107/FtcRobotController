package org.firstinspires.ftc.teamcode.fy23.controls.ctlpad;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.framework.ctlpad.CTL2Java.TestScheme;

/** A testing OpMode that prints TeleOpState to telemetry. */
@TeleOp
public class TeleOpStatePrinter23 extends OpMode {

    TeleOpScheme23 scheme;
//    private double driveSpeed;

    @Override
    public void init() {
        scheme = new TestScheme(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        TeleOpState23 state = scheme.getState();
        telemetry.addData("Max. Drive Speed", state.getMaxDriveSpeed());
        telemetry.addData("Drive", state.getDts().drive);
        telemetry.addData("Turn", state.getDts().turn);
        telemetry.addData("Strafe", state.getDts().strafe);
        telemetry.addData("Arm Pivot", state.getArmMovement());
        telemetry.addData("Elevator", state.getElevatorMovement());
        telemetry.addData("Claw State", state.getClawState());
        telemetry.addData("Launch Plane?", state.isLaunchPlane());
//        telemetry.addData("Drive Speed Up?", state.isDriveSpeedUp());
//        telemetry.addData("Drive Speed Down?", state.isDriveSpeedDown());

//        if (state.isDriveSpeedUp()) {
//            driveSpeed += 1;
//        } else if (state.isDriveSpeedDown()) {
//            driveSpeed -= 1;
//        }

//        telemetry.addData("Drive Speed", driveSpeed);
    }
}
