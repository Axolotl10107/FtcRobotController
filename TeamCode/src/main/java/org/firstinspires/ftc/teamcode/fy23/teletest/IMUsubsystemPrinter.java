package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;

@TeleOp(name="(RobotB) IMU Subsystem Printer", group="TeleTest")
public class IMUsubsystemPrinter extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
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
