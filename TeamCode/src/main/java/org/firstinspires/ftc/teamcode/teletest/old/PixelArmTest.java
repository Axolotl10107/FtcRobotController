package org.firstinspires.ftc.teamcode.teletest.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse24;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;

@Disabled
@TeleOp()
public class PixelArmTest extends OpMode {

    Robot24 robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot24(RobotRoundhouse24.getRobotAParams(hardwareMap), hardwareMap);
        telemetry.setMsTransmissionInterval(50);
        TelemetrySingleton.setInstance(telemetry);
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
