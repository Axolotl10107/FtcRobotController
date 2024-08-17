package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp()
public class RandomMotorTest extends OpMode {

    DcMotorEx testMotor;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotorEx.class, "armPivot");
    }

    @Override
    public void start() {
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        testMotor.setTargetPosition(0);
//        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        testMotor.setVelocity(400);
//        testMotor.setTargetPosition(-2000);
    }

    @Override
    public void loop() {
        double applyVel = 800 * -gamepad1.left_stick_x;
        testMotor.setVelocity(applyVel);
        telemetry.addData("Controller", -gamepad1.left_stick_x);
        telemetry.addData("Velocity", testMotor.getVelocity());
        telemetry.addData("Position", testMotor.getCurrentPosition());
    }
}
