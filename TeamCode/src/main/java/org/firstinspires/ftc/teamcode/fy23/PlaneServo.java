package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo_for_plane", group="TeleTest")
public class PlaneServo extends LinearOpMode {
    Servo servo123456789;

    @Override
    public void runOpMode() {
//initialize servo
        servo123456789 = hardwareMap.get(Servo.class, "planeservo");
        servo123456789.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper == true) {
                servo123456789.setPosition(1);

            }
            else {
                servo123456789.setPosition(0);
            }
        }
    }
}
