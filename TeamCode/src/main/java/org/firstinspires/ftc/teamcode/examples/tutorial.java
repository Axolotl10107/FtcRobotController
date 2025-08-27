package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="DiffrentName", group="WhoKnows")
public class tutorial extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() {
        hardwareMap.get(DcMotor.class,"motor");
        telemetry.addData("ready?", "gooooo!");
        telemetry.update();
        waitForStart();

        telemetry.addData("Hello", "Michael");
        telemetry.update();
        while(opModeIsActive()){
            motor.setPower(0.5);

        }


    }
}
