package org.firstinspires.ftc.teamcode.fy24.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

@Disabled
@TeleOp(name="Code for Old Claw", group="")
public class OldClawNewCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo servoClaw;
        servoClaw = hardwareMap.get(Servo.class, "clawServo");
        waitForStart();
        telemetry.update();
        double POS = 0.15;
        boolean buttonPressed = false;
        double speed = .03;

        while (opModeIsActive()) {
            buttonPressed = false;
            if (gamepad1.a && servoClaw.getPosition() > 0.15) {
                POS = servoClaw.getPosition() - speed;
                buttonPressed = true;
            }
            if (gamepad1.b && servoClaw.getPosition() < 0.24) {
                POS = servoClaw.getPosition() + speed;
                buttonPressed = true;
            }
            if (POS == 0.12) {
                POS = 0.15;
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
