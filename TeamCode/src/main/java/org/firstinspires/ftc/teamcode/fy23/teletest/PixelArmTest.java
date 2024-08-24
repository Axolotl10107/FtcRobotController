package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;

@TeleOp()
public class PixelArmTest extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
    }

    @Override
    public void loop() {
        robot.arm.setPivotPower(gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("Power", robot.arm.getPivotPower());
        telemetry.addData("Velocity", robot.arm.getPivotVelocity());
        telemetry.addData("Position", robot.arm.getPivotPosition());
        robot.update();
    }
}
