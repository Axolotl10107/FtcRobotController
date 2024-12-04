package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.TeleOpState23;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

@TeleOp(name="Robot B Test", group="")
public class RobotBTeleTest extends OpMode {

    Robot24 robot;
    IMUCorrector imuCorrector;
    FieldyTeleOpScheme23 controlScheme;
    double maxDrivePower = 1.0;
    double maxDrivePowerStep = 0.1;

    @Override
    public void init() {
        robot = new Robot24(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
        IMUCorrector.Parameters params = new IMUCorrector.Parameters(robot.imu, new TunablePID(robot.extendedParameters.hdgCorrectionPIDConsts));
        params.haveHitTargetToleranceDegrees = 0.1;
        params.hdgErrToleranceDegrees = 1.0;
        params.maxCorrectionPower = 0.1;
        params.turnPowerThreshold = 0.05;
        imuCorrector = new IMUCorrector(params);
        controlScheme = new FieldyTeleOpScheme23(gamepad1, gamepad2, robot.imu);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double currentHeading = robot.imu.yaw();
        TeleOpState23 controlState = controlScheme.getState();

        // MecanumDrive
        DTS correctedDTS = imuCorrector.correctDTS(controlState.getDts());
        DTS normalizedDTS = correctedDTS.normalize();
        DTS scaledDTS = normalizedDTS.scale(maxDrivePower);
        robot.drive.applyDTS(scaledDTS);

        // max. drive power
//        if (controlState.isDriveSpeedUp() && maxDrivePower < 0.9) {
//            maxDrivePower += maxDrivePowerStep;
//        } else if (controlState.isDriveSpeedDown() && maxDrivePower > 0.1) {
//            maxDrivePower -= maxDrivePowerStep;
//        }

        // IMUcorrector - square up
        if (controlState.isSquareUp()) {
            imuCorrector.squareUp();
        }

        // Claw
        robot.claw.setState(controlState.getClawState());

        // PixelArm
        robot.arm.setPivotPower(controlState.getArmMovement());
        robot.arm.setElevatorPower(controlState.getElevatorMovement());

        // PlaneLauncher
//        if (controlState.isLaunchPlane()) {
//            robot.planeLauncher.launch();
//        }

        robot.update();

        // telemetry
        telemetry.addData("Max. Drive Power", maxDrivePower);
        telemetry.addLine("-----------------------------------------------------");
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addLine("Some inaccurate information:");
        telemetry.addData("Drive", scaledDTS.drive);
        telemetry.addData("Turn", scaledDTS.turn);
        telemetry.addData("Strafe", scaledDTS.strafe);
    }

    @Override
    public void stop() {

    }
}
