package org.firstinspires.ftc.teamcode.fy23.robot.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy24.IndyTeleOpScheme24;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy24.TeleOpScheme24;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy24.TeleOpState24;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

@TeleOp(group="TeleTest")
public class RobotBTestTeleOp extends OpMode {

    Robot24 robot;
    TeleOpState24 controlsState;
    TeleOpScheme24 controlsScheme;
    IMUCorrector imuCorrector;

    @Override
    public void init() {
        robot = new Robot24(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
        controlsScheme = new IndyTeleOpScheme24(gamepad1, gamepad2);
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
        robot.drive.applyDTS( imuCorrector.correctDTS( normalizedDTS ) );
//        robot.drive.applyDTS(normalizedDTS);

        robot.arm.setPivotPower( controlsState.getArmMovement() );

        robot.arm.setElevatorPower( controlsState.getElevatorMovement() );

        if (controlsState.isSquareUp()) {
            imuCorrector.squareUp();
        }

        if (controlsState.isBrake()) {
            robot.drive.applyDTS( new DTS( 0, 0, 0 ) );
        }

        telemetry.addData("drive", normalizedDTS.drive);
        telemetry.addData("turn", normalizedDTS.turn);
        telemetry.addData("strafe", normalizedDTS.strafe);
        telemetry.addData("heading", robot.imu.yaw());
        telemetry.addData("heading error", imuCorrector.getHeadingError());
        telemetry.addData("haveHitTarget?", imuCorrector.hasHitTarget());
        telemetry.addData("turning?", imuCorrector.isTurning());
    }
}
