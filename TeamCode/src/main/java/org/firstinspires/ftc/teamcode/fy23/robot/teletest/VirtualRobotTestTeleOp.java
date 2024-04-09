package org.firstinspires.ftc.teamcode.fy23.robot.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.FieldyTeleOpScheme;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.TeleOpState;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUcorrector;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

@TeleOp
public class VirtualRobotTestTeleOp extends OpMode {

    Robot robot;
    TeleOpState controlsState;
    FieldyTeleOpScheme controlsScheme;
    IMUcorrector imuCorrector;

    @Override
    public void init() {
        robot = new Robot(RobotRoundhouse.getVirtualRobotParams(hardwareMap), hardwareMap);
        controlsScheme = new FieldyTeleOpScheme(gamepad1, gamepad2);
        imuCorrector = new IMUcorrector(hardwareMap, 0.023, 0, 0);
    }

    @Override
    public void loop() {
        robot.update();
        controlsState = controlsScheme.getState(Math.toRadians(robot.imu.yaw()));
        DTS normalizedDTS = controlsState.getDts().normalize();
        robot.drive.applyDTS(imuCorrector.correctDTS(normalizedDTS));
        robot.drive.applyDTS(normalizedDTS);

        if (gamepad1.x) {
            imuCorrector.squareUp();
        }

        telemetry.addData("drive", normalizedDTS.drive);
        telemetry.addData("turn", normalizedDTS.turn);
        telemetry.addData("strafe", normalizedDTS.strafe);
    }
}
