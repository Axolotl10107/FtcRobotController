package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;

@TeleOp()
public class AutomaticPixelArmTest extends LinearOpMode {

    Robot24 robot;
    double telemetryPower;
    Telemetry opModeTelemetry;

    private void updateTelemetry() {
        Telemetry telemetry = new MultipleTelemetry(opModeTelemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Set power", telemetryPower);
        telemetry.addData("Actual Power", robot.arm.getPivotPower());
        telemetry.addData("Actual Velocity", robot.arm.getPivotVelocity());
        telemetry.addData("Actual Position", robot.arm.getPivotPosition());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        opModeTelemetry = telemetry;

        robot = new Robot24(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);

        waitForStart();

        robot.arm.setPivotPower(0.5);
        telemetryPower = 0.5;
        while (robot.arm.getPivotPosition() < 2000 && opModeIsActive()) {
            robot.update();
            updateTelemetry();
        }

        robot.arm.setPivotPower(-0.5);
        telemetryPower = -0.5;
        while (robot.arm.getPivotPosition() > 0 && opModeIsActive()) {
            robot.update();
            updateTelemetry();
        }

        robot.arm.setPivotPower(1);
        telemetryPower = 1;
        while (robot.arm.getPivotPosition() < 2000 && opModeIsActive()) {
            robot.update();
            updateTelemetry();
        }

        robot.arm.setPivotPower(-1);
        telemetryPower = -1;
        while (robot.arm.getPivotPosition() > 0 && opModeIsActive()) {
            robot.update();
            updateTelemetry();
        }

        robot.arm.setPivotPower(0.25);
        telemetryPower = 0.25;
        while (robot.arm.getPivotPosition() < 2000 && opModeIsActive()) {
            robot.update();
            updateTelemetry();
        }

        robot.arm.setPivotPower(-0.25);
        telemetryPower = -0.25;
        while (robot.arm.getPivotPosition() > 0 && opModeIsActive()) {
            robot.update();
            updateTelemetry();
        }
    }

}
