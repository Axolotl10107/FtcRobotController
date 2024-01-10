package org.firstinspires.ftc.teamcode.fy23.robot.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.controls.FieldyGamepadLS;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadInputs;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadTrueDTS;
import org.firstinspires.ftc.teamcode.fy23.robot.DTSscaler;
import org.firstinspires.ftc.teamcode.fy23.robot.IMUcorrector;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotB;
import org.firstinspires.ftc.teamcode.fy23.robot.TunablePID;

@TeleOp
public class RobotBIMUDriveTuner extends OpMode {

    GamepadTrueDTS gamepad;
    IMUcorrector imuCorrector;
    DTSscaler scaler;
    RobotB robot;

    TunablePID pid;

    double changeAmount = 0.01;

    @Override
    public void init() {
//        imuCorrector = new IMUcorrector(hardwareMap, robot.pidConsts);
        scaler = new DTSscaler();
        robot = new RobotB(hardwareMap);
    }

    public void start() {
        imuCorrector = new IMUcorrector(hardwareMap, robot.pidConsts);
        // Why is this here? Because Virtual Robot is slow, I guess?
        pid = imuCorrector.pid;
        gamepad = new GamepadTrueDTS(gamepad1, gamepad2);
        // totally a safety mechanism - try moving during init without a gamepad :)
    }

    @Override
    public void loop() {
        robot.drive.applyDTS(scaler.scale(imuCorrector.correctDTS(gamepad.dts())));
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
            imuCorrector.targetHeading += 1;
        }
        if (gamepad.hdgDown()) {
            imuCorrector.targetHeading -= 1;
        }

        telemetry.addData("Change by", changeAmount);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Requested turn", gamepad.dts().turn);
        telemetry.addData("Corrected turn", imuCorrector.correctedTurnPower);
        telemetry.addData("Actual turn", scaler.scaledTurn);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Proportional", pid.proportional);
        telemetry.addData("Integral", pid.integral);
        telemetry.addData("Integral Multiplier", pid.integralMultiplier);
        telemetry.addData("Derivative", pid.derivative);
        telemetry.addData("Derivative Multiplier", pid.derivativeMultiplier);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Current Heading", imuCorrector.imu.yaw());
        telemetry.addData("Target Heading", imuCorrector.targetHeading);
        telemetry.addData("Heading Error", imuCorrector.headingError);
        telemetry.addData("Last Error", imuCorrector.lastError);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("leftFront encoder", robot.drive.leftFront.getCurrentPosition());
        telemetry.addData("rightFront encoder", robot.drive.rightFront.getCurrentPosition());
        telemetry.addData("leftBack encoder", robot.drive.leftBack.getCurrentPosition());
        telemetry.addData("rightBack encoder", robot.drive.rightBack.getCurrentPosition());
    }
}
