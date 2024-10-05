package org.firstinspires.ftc.teamcode.fy24.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

@TeleOp(name="Code for Gripper Claw", group="")
public class Code_Servo_Claw extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo servoClaw;
        servoClaw = hardwareMap.get(Servo.class, "clawServo");
        waitForStart();
        telemetry.update();
        double POS = 0;
        boolean buttonPressed = false;
        double speed = .05;

        while (opModeIsActive()) {
            buttonPressed = false;
            if (gamepad1.a) {
                POS = servoClaw.getPosition() - speed;
                buttonPressed = true;
            }
            if (gamepad1.b) {
                POS = servoClaw.getPosition() + speed;
                buttonPressed = true;
            }
//            if (!buttonPressed) {
//                servoClaw.setPosition(POS);
//            }
            servoClaw.setPosition(POS);

            telemetry.addData("position", servoClaw.getPosition());
            telemetry.update();
            telemetry.update();

            Thread.sleep(100);
        }
    }
}
