package org.firstinspires.ftc.teamcode.fy24.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

@Disabled
@TeleOp(name="Active Intake Servo", group="")
public class ActiveIntakeServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo servoClaw;
        servoClaw = hardwareMap.get(CRServo.class, "clawServo");
        waitForStart();

        while  (opModeIsActive()) {
            if (gamepad1.a) {
                servoClaw.setPower(1);
            } else if (gamepad1.b) {
                servoClaw.setPower(-1);
            } else if (gamepad1.atRest()) {
                servoClaw.setPower(0);
            }
            telemetry.addData("Power: ", servoClaw.getPower());
            telemetry.update();
        }
    }
}
