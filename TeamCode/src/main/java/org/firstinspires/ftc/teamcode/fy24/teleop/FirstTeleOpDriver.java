package org.firstinspires.ftc.teamcode.fy24.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy24.controls.IndyTeleOpScheme24;
import org.firstinspires.ftc.teamcode.fy24.controls.TeleOpScheme24;
import org.firstinspires.ftc.teamcode.fy24.controls.TeleOpState24;
import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.framework.units.DTS;

@TeleOp(group="TeleTest")
public class FirstTeleOpDriver extends OpMode {

    Robot24 robot;
    TeleOpState24 controlsState;
    TeleOpScheme24 controlsScheme;
    IMUCorrector imuCorrector;

    @Override
    public void init() {
        robot = new Robot24(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
        controlsScheme = new IndyTeleOpScheme24(gamepad1, gamepad2);

        IMUCorrector.Parameters params = new IMUCorrector.Parameters(robot.imu, new TunablePID(robot.extendedParameters.hdgCorrectionPIDConsts));
        // IMUcorrector has other parameters, but they already have good defaults and don't usually need to be changed.
        imuCorrector = new IMUCorrector(params);
    }

    @Override
    public void loop() {
        robot.update();
        controlsState = controlsScheme.getState();
        DTS normalizedDTS = controlsState.getDts().normalize();
        robot.drive.applyDTS(imuCorrector.correctDTS(normalizedDTS));
        robot.drive.applyDTS(normalizedDTS);

        if (controlsState.isSquareUp()) {
            imuCorrector.squareUp();
        }

        }

    }

