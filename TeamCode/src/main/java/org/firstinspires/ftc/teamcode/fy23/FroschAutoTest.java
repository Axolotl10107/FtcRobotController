package org.firstinspires.ftc.teamcode.fy23;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.fy23.autoSwitch.AutoSequenceSwitcher;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.FieldyTeleOpScheme;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.*;

@Autonomous(name="FroschAutoTest", group="")
public class FroschAutoTest extends LinearOpMode {

    Robot robot;
    IMUCorrector imuCorrector;
    FieldyTeleOpScheme controlScheme;
    double maxDrivePower = 1.0;
    double maxDrivePowerStep = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.update();
        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
        IMUCorrector.Parameters params = new IMUCorrector.Parameters(robot.imu, new TunablePID(robot.extendedParameters.hdgCorrectionPIDConsts));
//        params.haveHitTargetToleranceDegrees = 0.1;
//        params.hdgErrToleranceDegrees = 1.0;
//        params.maxCorrectionPower = 0.1;
//        params.turnPowerThreshold = 0.05;

        imuCorrector = new IMUCorrector(params);
        controlScheme = new FieldyTeleOpScheme(gamepad1, gamepad2, robot.imu);

        Pose2d startPose = new Pose2d(0, 0, 0);
        TrajectorySequence seq1 = robot.drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(20)
                .build();

        TrajectorySequence seq2 = robot.drive.trajectorySequenceBuilder(startPose)
                .strafeRight(20)
                .build();

        TrajectorySequence leftRight1 = robot.drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(10)
                .waitSeconds(1)
                .strafeRight(10)
                .waitSeconds(1)
                .strafeRight(10)
                .build();

        TrajectorySequence leftRight5 = robot.drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(10)
                .waitSeconds(5)
                .strafeRight(10)
                .waitSeconds(5)
                .strafeRight(10)
                .build();

        AutoSequenceSwitcher switcher = new AutoSequenceSwitcher();
        switcher.addSequence("left", seq1);
        switcher.addSequence("right", seq2);
        switcher.addSequence("left-right 1", leftRight1);
        switcher.addSequence("left-right 5", leftRight5);

        boolean lock = false;
        while (!gamepad1.a) {
            if (gamepad1.dpad_up && !lock) {
                switcher.selectNext();
                lock = true;
            } else if (gamepad1.dpad_down && !lock) {
                switcher.selectPrevious();
                lock = true;
            } else if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                lock = false;
            }
            telemetry.addData("selected:", switcher.getSelected().getName());
            telemetry.addLine("press A to lock");
            telemetry.update();
        }
        telemetry.addLine("selection locked");
        telemetry.update();

        waitForStart();

        robot.drive.followTrajectorySequence(switcher.getSelected().getTrajectorySequence());
//        robot.update();
    }


}
