package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RunGamePadMotor", group="TeleTest")
public class RunGamePadMotor extends LinearOpMode {
    private DcMotor OneMotor;
    @Override
    public void runOpMode() {
        OneMotor = hardwareMap.get(DcMotor.class, "OneMotor");
        waitForStart();
        while(opModeIsActive()) {
            OneMotor.setPower(gamepad1.left_stick_y);
        }
    }
}

