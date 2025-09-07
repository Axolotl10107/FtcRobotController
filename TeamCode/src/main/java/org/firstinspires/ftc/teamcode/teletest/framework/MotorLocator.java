package org.firstinspires.ftc.teamcode.teletest.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;

@TeleOp(name="Motor Locator", group="TeleTest")
public class MotorLocator extends OpMode {

    Robot25 robot;

    @Override
    public void init() {
        robot = new Robot25(RobotRoundhouse25.getRobotBParams(hardwareMap), hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            robot.drive.getLeftFrontMotor().setPower(1);
        } else {
            robot.drive.getLeftFrontMotor().setPower(0);
        }

        if (gamepad1.dpad_down) {
            robot.drive.getLeftBackMotor().setPower(1);
        } else {
            robot.drive.getLeftBackMotor().setPower(0);
        }

        if (gamepad1.y) {
            robot.drive.getRightFrontMotor().setPower(1);
        } else {
            robot.drive.getRightFrontMotor().setPower(0);
        }

        if (gamepad1.a) {
            robot.drive.getRightBackMotor().setPower(1);
        } else {
            robot.drive.getRightBackMotor().setPower(0);
        }

        robot.update();

        telemetry.addData("leftFront", "D-Pad Up");
        telemetry.addData("leftBack", "D-Pad Down");
        telemetry.addData("rightFront", "Y");
        telemetry.addData("rightBack", "A");
        telemetry.update();
    }
}
