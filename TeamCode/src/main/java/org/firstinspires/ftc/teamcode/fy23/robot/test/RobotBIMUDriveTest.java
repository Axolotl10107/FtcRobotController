package org.firstinspires.ftc.teamcode.fy23.robot.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.controls.GamepadTrueDTS;
import org.firstinspires.ftc.teamcode.fy23.robot.DTSscaler;
import org.firstinspires.ftc.teamcode.fy23.robot.IMUcorrector;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotB;

@TeleOp
public class RobotBIMUDriveTest extends OpMode {

    GamepadTrueDTS gamepad;
    IMUcorrector imuCorrector;
    DTSscaler scaler;
    RobotB robot;

    @Override
    public void init() {
        gamepad = new GamepadTrueDTS(gamepad1, gamepad2);
        imuCorrector = new IMUcorrector(hardwareMap, robot.pidConsts);
        scaler = new DTSscaler();
        robot = new RobotB(hardwareMap);
    }

    @Override
    public void loop() {
        robot.drive.applyDTS(scaler.scale(imuCorrector.correctDTS(gamepad.dts())));
        // Yup. That's the OpMode. That line lets you drive the robot.

        telemetry.addData("Requested turn", gamepad.dts().turn);
        telemetry.addData("Corrected turn", imuCorrector.correctedTurnPower);
        telemetry.addData("Actual turn", scaler.scaledTurn);
    }
}
