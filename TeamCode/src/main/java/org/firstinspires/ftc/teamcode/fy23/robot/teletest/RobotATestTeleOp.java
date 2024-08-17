package org.firstinspires.ftc.teamcode.fy23.robot.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.FieldyTeleOpScheme;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.TeleOpScheme;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.TeleOpState;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUcorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

@TeleOp(group="TeleTest")
public class RobotATestTeleOp extends OpMode {

    Robot robot;
    TeleOpState controlsState;
    TeleOpScheme controlsScheme;
    IMUcorrector imuCorrector;

    @Override
    public void init() {
        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
        controlsScheme = new FieldyTeleOpScheme(gamepad1, gamepad2, robot.imu);

        IMUcorrector.Parameters params = new IMUcorrector.Parameters();
        params.imu = robot.imu;
        params.pid = new TunablePID(robot.hdgCorrectionPIDconsts);
        // IMUcorrector has other parameters, but they already have good defaults and don't usually need to be changed.
        imuCorrector = new IMUcorrector(params);
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

        if (controlsState.isLaunchPlane()) {
            robot.planeLauncher.launch();
        }

        telemetry.addData("drive", normalizedDTS.drive);
        telemetry.addData("turn", normalizedDTS.turn);
        telemetry.addData("strafe", normalizedDTS.strafe);
        telemetry.addData("heading", robot.imu.yaw());
        telemetry.addData("heading error", imuCorrector.getHeadingError());
        telemetry.addData("haveHitTarget?", imuCorrector.haveHitTarget());
        telemetry.addData("turning?", imuCorrector.isTurning());
        telemetry.addLine("----------------------");
        telemetry.addData("arm position", robot.arm.getPivotPosition());
        telemetry.addData("elevator position", robot.arm.getElevatorPosition());
    }
}
