package org.firstinspires.ftc.teamcode.old_meddev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTest", group="TeleTest")
public class MotorTest extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private double power;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.start) {
                power = 1;
            } else {
                power = 0.5;
            }
            if (gamepad1.dpad_up) {
                leftFront.setPower(power);
            } else {
                leftFront.setPower(0);
            } if (gamepad1.dpad_down) {
                leftBack.setPower(power);
            } else {
                leftBack.setPower(0);
            } if (gamepad1.y) {
                rightFront.setPower(power);
            } else {
                rightFront.setPower(0);
            } if (gamepad1.a) {
                rightBack.setPower(power);
            } else {
                rightBack.setPower(0);
            }
        }
    }
}
