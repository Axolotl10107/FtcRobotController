package org.firstinspires.ftc.teamcode.fy23.robot.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.TeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.TeleOpState23;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

@TeleOp(group="TeleTest")
public class VirtualRobotTestTeleOp extends OpMode {

    Robot24 robot;
    TeleOpState23 controlsState;
    TeleOpScheme23 controlsScheme;
    IMUCorrector imuCorrector;

    @Override
    public void init() {
        robot = new Robot24(RobotRoundhouse.getParamsAuto(hardwareMap), hardwareMap);
        controlsScheme = new FieldyTeleOpScheme23(gamepad1, gamepad2, robot.imu);
        IMUCorrector.Parameters params = new IMUCorrector.Parameters(robot.imu, new TunablePID(robot.extendedParameters.hdgCorrectionPIDConsts));
        params.haveHitTargetToleranceDegrees = 0.1;
        params.hdgErrToleranceDegrees = 1.0;
        params.maxCorrectionPower = 0.1;
        params.turnPowerThreshold = 0.05;
        imuCorrector = new IMUCorrector(params);
    }

    @Override
    public void loop() {
        robot.update();
        controlsState = controlsScheme.getState();
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
