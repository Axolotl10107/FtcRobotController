package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RunOneMotor", group="TeleTest")
public class RunOneMotor extends LinearOpMode {
    private DcMotor OneMotor;
    @Override
    public void runOpMode() {
        OneMotor = hardwareMap.get(DcMotor.class, "OneMotor");
        waitForStart();
        OneMotor.setPower(.5);
        while(opModeIsActive()) {
        }
    }
}
