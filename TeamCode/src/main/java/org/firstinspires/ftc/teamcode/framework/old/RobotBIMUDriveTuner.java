package org.firstinspires.ftc.teamcode.framework.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadTrueDTS;
import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.framework.units.PIDConsts;

import java.io.File;

@Disabled
@TeleOp
@Deprecated
public class RobotBIMUDriveTuner extends OpMode {

    GamepadTrueDTS gamepad;
    IMUCorrector imuCorrector;
    DTSscaler scaler;
    Robot24 robot;

    TunablePID pid;

    double changeAmount = 0.01;

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
        gamepad = new GamepadTrueDTS(gamepad1, gamepad2);
        // totally a safety mechanism - try moving during init without a gamepad :)
    }

    @Override
    public void loop() {
        robot.drive.applyDTS(scaler.scale(imuCorrector.correctDTS(gamepad.dts())));
        // Yup. That's the OpMode. That line lets you drive the robot.

        if (gamepad.pUp()) {
            pid.setkP(pid.kP() + changeAmount);
        }
        if (gamepad.pDown()) {
            pid.setkP(pid.kP() - changeAmount);
        }
        if (gamepad.imUp()) {
            pid.setkI(pid.kI() + changeAmount);
        }
        if (gamepad.imDown()) {
            pid.setkI(pid.kI() - changeAmount);
        }
        if (gamepad.dmUp()) {
            pid.setkD(pid.kD() + changeAmount);
        }
        if (gamepad.dmDown()) {
            pid.setkD(pid.kD() - changeAmount);
        }

        if (gamepad.changeUp()) {
            changeAmount *= 10;
        }
        if (gamepad.changeDown()) {
            changeAmount /= 10;
        }

        if (gamepad.hdgUp()) {
            imuCorrector.setTargetHeading(imuCorrector.getTargetHeading() + 1);
        }
        if (gamepad.hdgDown()) {
            imuCorrector.setTargetHeading(imuCorrector.getTargetHeading() - 1);
        }

        if (gamepad.save()) {
            // modified from SensorBNO055IMUCalibration example
            String filename = "RobotB.pid";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            PIDConsts constsToWrite = new PIDConsts(pid.kP(), pid.kI(), pid.maxI(), pid.kD());
            ReadWriteFile.writeFile(file, constsToWrite.serialize());
            telemetry.log().add("saved to '%s'", filename);
        }

        telemetry.addLine("D-Pad up/down - proportional");
        telemetry.addLine("Y/A - integral multiplier");
        telemetry.addLine("B/X - derivative multiplier");
        telemetry.addLine("Start/Back - amount of change");
        telemetry.addData("Change by", changeAmount);
        telemetry.addLine("D-Pad right/left - heading (ignores change amount)");
        telemetry.addLine("Right Bumper - save to RobotB.pid");
        telemetry.addLine("-------------------------------------");
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
