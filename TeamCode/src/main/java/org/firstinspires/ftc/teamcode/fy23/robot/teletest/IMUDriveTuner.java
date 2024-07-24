package org.firstinspires.ftc.teamcode.fy23.robot.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.specific.imudrivetuner.IMUDriveTunerScheme;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.specific.imudrivetuner.IMUDriveTunerState;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUcorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.units.PIDconsts;

import java.io.File;

@TeleOp(group="TeleTest")
public class IMUDriveTuner extends OpMode {

    IMUDriveTunerState gamepad;
    IMUDriveTunerScheme controlScheme;
    IMUcorrector imuCorrector;
    Robot robot;
    TunablePID pid;

    double changeAmount = 0.01;

    @Override
    public void init() {
//        imuCorrector = new IMUcorrector(hardwareMap, robot.pidConsts);
        // TODO: Detect and/or parameterize which robot to use
        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
    }

    public void start() {
        IMUcorrector.Parameters params = new IMUcorrector.Parameters();
        params.haveHitTargetTolerance = 0.1;
        params.hdgErrTolerance = 1.0;
        params.maxCorrection = 0.1;
        params.turnThreshold = 0.05;
        params.imu = robot.imu;
        params.pid = new TunablePID(robot.hdgCorrectionPIDconsts);
        params.errorSampleTimer = new ElapsedTime();
        params.errorSampleDelay = 1150;
        imuCorrector = new IMUcorrector(params);
        // Why is this down here instead of in init()? Because virtual_robot is slow, I guess?
        pid = params.pid;
        controlScheme = new IMUDriveTunerScheme(gamepad1, gamepad2);
        // totally a safety mechanism - try moving during init without a gamepad :)
    }

    @Override
    public void loop() {
        gamepad = controlScheme.getState();

        robot.drive.applyDTS(imuCorrector.correctDTS(gamepad.getDts()).normalize());
        // Yup. That's the OpMode. That line lets you drive the robot.

        if (gamepad.pUp()) {
            pid.setProportional(pid.getProportional() + changeAmount);
        }
        if (gamepad.pDown()) {
            pid.setProportional(pid.getProportional() - changeAmount);
        }
        if (gamepad.imUp()) {
            pid.setIntegralMultiplier(pid.getIntegralMultiplier() + changeAmount);
        }
        if (gamepad.imDown()) {
            pid.setIntegralMultiplier(pid.getIntegralMultiplier() - changeAmount);
        }
        if (gamepad.dmUp()) {
            pid.setDerivativeMultiplier(pid.getDerivativeMultiplier() + changeAmount);
        }
        if (gamepad.dmDown()) {
            pid.setDerivativeMultiplier(pid.getDerivativeMultiplier() - changeAmount);
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
            // TODO: Detect robot and/or parameterize filename
            String filename = "RobotA.pid";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            PIDconsts constsToWrite = new PIDconsts(pid.getProportional(), pid.getIntegralMultiplier(), pid.getDerivativeMultiplier());
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
        telemetry.addData("Requested turn", gamepad.getDts().turn);
        telemetry.addData("Actual turn", gamepad.getDts().normalize().turn);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Proportional", pid.getProportional());
        telemetry.addData("Integral", pid.getIntegral());
        telemetry.addData("Integral Multiplier", pid.getIntegralMultiplier());
        telemetry.addData("Derivative", pid.getDerivative());
        telemetry.addData("Derivative Multiplier", pid.getDerivativeMultiplier());
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Current Heading", robot.imu.yaw());
        telemetry.addData("Target Heading", imuCorrector.getTargetHeading());
        telemetry.addData("Heading Error", imuCorrector.getHeadingError());
        telemetry.addData("Last Error", imuCorrector.getLastHeadingError());
        telemetry.addLine("-------------------------------------");
        telemetry.addData("leftFront encoder", robot.drive.leftFront.getCurrentPosition());
        telemetry.addData("rightFront encoder", robot.drive.rightFront.getCurrentPosition());
        telemetry.addData("leftBack encoder", robot.drive.leftBack.getCurrentPosition());
        telemetry.addData("rightBack encoder", robot.drive.rightBack.getCurrentPosition());
    }
}
