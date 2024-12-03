//ALoTO 2022-23
//Start + A sets controller to Gamepad 1
//Start + B sets controller to Gamepad 2
package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name="ServoTeleTestOld", group="TeleTest")

public class ServoTeleTestOld extends LinearOpMode {
    private Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo1");
        servo.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
//We still need to code this
        while (opModeIsActive()) {
            telemetry.addData("Servo Position", servo.getPosition());
            if (gamepad1.left_bumper) {
                servo.setPosition(0);
            } else if (gamepad1.right_bumper) {
                servo.setPosition(1);
            } else {
                servo.setPosition((gamepad1.left_stick_x / 2) + 0.5);
                //+ 0.5 - stick goes -1 to 1, servo goes 0 to 1. This offsets the stick range.
                //Div. stick position by 2, so that, with the offset, 0 and 1 are at edges of stick
            }
            telemetry.update();
        }
    }
}
