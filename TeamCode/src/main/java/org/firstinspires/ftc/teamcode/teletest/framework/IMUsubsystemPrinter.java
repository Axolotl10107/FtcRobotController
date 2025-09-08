package org.firstinspires.ftc.teamcode.teletest.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;

@TeleOp(name="IMU Subsystem Printer", group="TeleTest")
public class IMUsubsystemPrinter extends OpMode {

    Robot25 robot;

    @Override
    public void init() {
        try {
            robot = new Robot25(RobotRoundhouse25.getParamsAuto(hardwareMap), hardwareMap);
        } catch (Robot25.InvalidDeviceClassException | RobotRoundhouse25.OldRobotException e) {
            throw new RuntimeException(e);
        }
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
