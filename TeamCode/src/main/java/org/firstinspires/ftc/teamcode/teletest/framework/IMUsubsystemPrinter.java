package org.firstinspires.ftc.teamcode.teletest.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse;

@TeleOp(name="(RobotB) IMU Subsystem Printer", group="TeleTest")
public class IMUsubsystemPrinter extends OpMode {

    Robot24 robot;

    @Override
    public void init() {
        robot = new Robot24(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Yaw", robot.imu.yaw());
        telemetry.addData("Yaw Velocity", robot.imu.yawVel());
        telemetry.addData("Roll", robot.imu.roll());
        telemetry.addData("Roll Velocity", robot.imu.rollVel());
        telemetry.addData("Pitch", robot.imu.pitch());
        telemetry.addData("Pitch Velocity", robot.imu.pitchVel());
    }
}
