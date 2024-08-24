package org.firstinspires.ftc.teamcode.fy23.robot.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;

@TeleOp()
public class MotorLocator extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            robot.drive.leftFront.setPower(1);
        } else {
            robot.drive.leftFront.setPower(0);
        }

        if (gamepad1.dpad_down) {
            robot.drive.leftBack.setPower(1);
        } else {
            robot.drive.leftBack.setPower(0);
        }

        if (gamepad1.y) {
            robot.drive.rightFront.setPower(1);
        } else {
            robot.drive.rightFront.setPower(0);
        }

        if (gamepad1.b) {
            robot.drive.rightBack.setPower(1);
        } else {
            robot.drive.rightBack.setPower(0);
        }

        robot.update();
    }
}
