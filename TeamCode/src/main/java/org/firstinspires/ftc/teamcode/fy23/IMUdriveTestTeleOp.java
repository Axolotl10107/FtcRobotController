package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.controls.GamepadDTS;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadInterface;

@TeleOp
public class IMUdriveTestTeleOp extends OpMode {
    IMUdrive drive;
    GamepadInterface controls;

    @Override
    public void init() {
        drive = new IMUdrive(hardwareMap, telemetry);
        controls = new GamepadDTS(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        drive.setAllPowers(controls.forwardMovement(), -controls.rotateMovement(), controls.strafeMovement());
        drive.update();
    }
}
