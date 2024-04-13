package org.firstinspires.ftc.teamcode.fy23.robot.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.controls.FieldyGamepadLS;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUcorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.FriendlyIMUImpl;

@Disabled
@TeleOp
public class RobotBIMUDriveTest extends OpMode {

    FieldyGamepadLS gamepad;
    IMUcorrector imuCorrector;
    DTSscaler scaler;
    Robot robot;

    TunablePID pid;

    @Override
    public void init() {
//        imuCorrector = new IMUcorrector(hardwareMap, robot.pidConsts);
        scaler = new DTSscaler();
        robot = new Robot(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
    }

    public void start() {
        IMUcorrector.Parameters params = new IMUcorrector.Parameters();
        params.haveHitTargetTolerance = 0.1;
        params.hdgErrTolerance = 1.0;
        params.maxCorrection = 0.1;
        params.turnThreshold = 0.05;
        params.imu = robot.imu;
        params.pid = new TunablePID(robot.hdgCorrectionPIDconsts);
        imuCorrector = new IMUcorrector(params);
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
        telemetry.addData("Proportional", pid.getProportional());
        telemetry.addData("Integral", pid.getIntegral());
        telemetry.addData("Integral Multiplier", pid.getIntegralMultiplier());
        telemetry.addData("Derivative", pid.getDerivative());
        telemetry.addData("Derivative Multiplier", pid.getDerivativeMultiplier());
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Current Heading", robot.imu.yaw());
        telemetry.addData("Target Heading", imuCorrector.targetHeading);
        telemetry.addData("Heading Error", imuCorrector.headingError);
        telemetry.addData("Last Error", imuCorrector.lastHdgError);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("leftFront encoder", robot.drive.leftFront.getCurrentPosition());
        telemetry.addData("rightFront encoder", robot.drive.rightFront.getCurrentPosition());
        telemetry.addData("leftBack encoder", robot.drive.leftBack.getCurrentPosition());
        telemetry.addData("rightBack encoder", robot.drive.rightBack.getCurrentPosition());
    }
}
