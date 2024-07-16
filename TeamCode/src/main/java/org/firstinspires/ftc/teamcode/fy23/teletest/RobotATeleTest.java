package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.FieldyTeleOpScheme;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.TeleOpState;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUcorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

@TeleOp
public class RobotATeleTest extends OpMode {

    Robot robot;
    IMUcorrector imuCorrector;
    FieldyTeleOpScheme controlScheme;
    double maxDrivePower = 1.0;
    double maxDrivePowerStep = 0.1;

    @Override
    public void init() {
        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
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
        controlScheme = new FieldyTeleOpScheme(gamepad1, gamepad2);
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
        TeleOpState controlState = controlScheme.getState(currentHeading);

        // MecanumDrive
        DTS correctedDTS = imuCorrector.correctDTS(controlState.getDts());
        DTS normalizedDTS = correctedDTS.normalize();
        DTS scaledDTS = normalizedDTS.scale(maxDrivePower);
        robot.drive.applyDTS(scaledDTS);

        // max. drive power
        if (controlState.isDriveSpeedUp() && maxDrivePower < 0.9) {
            maxDrivePower += maxDrivePowerStep;
        } else if (controlState.isDriveSpeedDown() && maxDrivePower > 0.1) {
            maxDrivePower -= maxDrivePowerStep;
        }

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
        if (controlState.isLaunchPlane()) {
            robot.planeLauncher.launch();
        }

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
