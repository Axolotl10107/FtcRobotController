package org.firstinspires.ftc.teamcode.framework.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.fy23.controls.FieldyGamepadLS;
import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse;

@Disabled
@TeleOp
@Deprecated
public class RobotBIMUDriveTest extends OpMode {

    FieldyGamepadLS gamepad;
    IMUCorrector imuCorrector;
    DTSscaler scaler;
    Robot24 robot;

    TunablePID pid;

    @Override
    public void init() {
//        imuCorrector = new IMUcorrector(hardwareMap, robot.pidConsts);
        scaler = new DTSscaler();
        robot = new Robot24(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
    }

    public void start() {
        IMUCorrector.Parameters params = new IMUCorrector.Parameters(robot.imu, new TunablePID(robot.extendedParameters.hdgCorrectionPIDConsts));
        params.haveHitTargetToleranceDegrees = 0.1;
        params.hdgErrToleranceDegrees = 1.0;
        params.maxCorrectionPower = 0.1;
        params.turnPowerThreshold = 0.05;
        imuCorrector = new IMUCorrector(params);
        // Why is this here? Because Virtual Robot is slow, I guess?
        pid = params.pid;
        gamepad = new FieldyGamepadLS(gamepad1, gamepad2, (FriendlyIMUImpl) robot.imu);
        // totally a safety mechanism - try moving during init without a gamepad :)
    }

    @Override
    public void loop() {
        robot.drive.applyDTS(scaler.scale(imuCorrector.correctDTS(gamepad.dts())));
        // Yup. That's the OpMode. That line lets you drive the robot.

//        if (gamepad.pUp()) {
//            pid.setProportional(pid.getProportional() + 0.01);
//        }
//        if (gamepad.pDown()) {
//            pid.setProportional(pid.getProportional() - 0.01);
//        }
//        if (gamepad.imUp()) {
//            pid.setIntegralMultiplier(pid.getIntegralMultiplier() + 0.01);
//        }
//        if (gamepad.imDown()) {
//            pid.setIntegralMultiplier(pid.getIntegralMultiplier() - 0.01);
//        }
//        if (gamepad.dmUp()) {
//            pid.setDerivativeMultiplier(pid.getDerivativeMultiplier() + 0.01);
//        }
//        if (gamepad.dmDown()) {
//            pid.setDerivativeMultiplier(pid.getDerivativeMultiplier() - 0.01);
//        }
//        if (gamepad.hdgUp()) {
//            imuCorrector.targetHeading += 1;
//        }
//        if (gamepad.hdgDown()) {
//            imuCorrector.targetHeading -= 1;
//        }

        telemetry.addData("Requested turn", gamepad.dts().turn);
//        telemetry.addData("Corrected turn", imuCorrector.correctedTurnPower);
        telemetry.addData("Actual turn", scaler.scaledTurn);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Proportional", pid.kP());
        telemetry.addData("Integral", pid.currentIntegralValue());
        telemetry.addData("Integral Multiplier", pid.kI());
        telemetry.addData("Derivative Multiplier", pid.kD());
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Current Heading", robot.imu.yaw());
        telemetry.addData("Target Heading", imuCorrector.getTargetHeading());
        telemetry.addData("Heading Error", imuCorrector.getHeadingError());
        telemetry.addData("Last Error", imuCorrector.getLastHeadingError());
        telemetry.addLine("-------------------------------------");
        telemetry.addData("leftFront encoder", robot.drive.getLeftFrontMotor().getCurrentPosition());
        telemetry.addData("rightFront encoder", robot.drive.getRightFrontMotor().getCurrentPosition());
        telemetry.addData("leftBack encoder", robot.drive.getLeftBackMotor().getCurrentPosition());
        telemetry.addData("rightBack encoder", robot.drive.getRightBackMotor().getCurrentPosition());
    }
}
