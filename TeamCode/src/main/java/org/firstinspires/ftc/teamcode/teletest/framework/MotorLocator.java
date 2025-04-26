package org.firstinspires.ftc.teamcode.teletest.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse;

@TeleOp()
public class MotorLocator extends OpMode {

    Robot24 robot;

    @Override
    public void init() {
        robot = new Robot24(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
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

        if (gamepad1.b) {
            robot.drive.getRightBackMotor().setPower(1);
        } else {
            robot.drive.getRightBackMotor().setPower(0);
        }

        robot.update();
    }
}
