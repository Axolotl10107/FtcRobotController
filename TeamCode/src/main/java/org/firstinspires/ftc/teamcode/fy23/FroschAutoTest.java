package org.firstinspires.ftc.teamcode.fy23;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.teamcode.fy23.autoSwitch.AutoSequenceSwitcher;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.FieldyTeleOpScheme;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.*;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive;

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
//        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
//        robot = new Robot(RobotRoundhouse.getParamsAuto(ControlHubDeviceNameManager.getControlHubDeviceNameManager().getDeviceName(), hardwareMap), hardwareMap);
        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
        RRMecanumDrive drive = robot.drive;

//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);
//        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startPose)
//                .strafeLeft(20)
//                .build();
//
//        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(startPose)
//                .strafeRight(20)
//                .build();
//
//        TrajectorySequence leftRight1 = drive.trajectorySequenceBuilder(startPose)
//                .strafeLeft(10)
//                .waitSeconds(1)
//                .strafeRight(10)
//                .waitSeconds(1)
//                .strafeRight(10)
//                .build();
//
//        TrajectorySequence leftRight5 = drive.trajectorySequenceBuilder(startPose)
//                .strafeLeft(10)
//                .waitSeconds(5)
//                .strafeRight(10)
//                .waitSeconds(5)
//                .strafeRight(10)
//                .build();
//
//        TrajectorySequence forwardBack = drive.trajectorySequenceBuilder(startPose)
//                .forward(10)
//                .back(30)
//                .forward(20)
//                .build();

        AutoSequenceSwitcher switcher = new AutoSequenceSwitcher();
        drive.setPoseEstimate(new Pose2d(-24*2, -24*2, 0));

        switcher.addSequence("field tour", drive.trajectorySequenceBuilder(new Pose2d(-24*2, -24*2, 0))
                .lineTo(new Vector2d(24*2, -24*2))
                .lineTo(new Vector2d(24*2, 24*2))
                .lineTo(new Vector2d(-24*2, 24*2))
                .lineTo(new Vector2d(-24*2, -24*2))
                .build()
        );

        boolean lock = false;
        while (!gamepad1.a && !isStarted()) {
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

        drive.followTrajectorySequence(switcher.getSelected().getTrajectorySequence());
//        update();
    }


}
