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
public class RobotATestTeleOp extends OpMode {

    Robot24 robot;
    TeleOpState23 controlsState;
    TeleOpScheme23 controlsScheme;
    IMUCorrector imuCorrector;

    @Override
    public void init() {
        robot = new Robot24(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
        controlsScheme = new FieldyTeleOpScheme23(gamepad1, gamepad2, robot.imu);

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

        robot.arm.setPivotPower(controlsState.getArmMovement());
        robot.arm.setElevatorPower(controlsState.getElevatorMovement());

//        if (controlsState.isLaunchPlane()) {
//            robot.planeLauncher.launch();
//        }

        robot.claw.setState(controlsState.getClawState());

        telemetry.addData("drive", normalizedDTS.drive);
        telemetry.addData("turn", normalizedDTS.turn);
        telemetry.addData("strafe", normalizedDTS.strafe);
        telemetry.addData("heading", robot.imu.yaw());
        telemetry.addData("heading error", imuCorrector.getHeadingError());
        telemetry.addData("haveHitTarget?", imuCorrector.hasHitTarget());
        telemetry.addData("turning?", imuCorrector.isTurning());
        telemetry.addLine("----------------------");
        telemetry.addData("arm position", robot.arm.getPivotPosition());
        telemetry.addData("elevator position", robot.arm.getElevatorPosition());
    }
}
