package org.firstinspires.ftc.teamcode.fy23;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.fy23.autoSwitch.AutoSequenceSwitcher;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive;

@Autonomous(name="FroschAutoTest", group="")
public class FroschAutoTest extends LinearOpMode {

    Robot24 robot;
    IMUCorrector imuCorrector;
    FieldyTeleOpScheme23 controlScheme;
    double maxDrivePower = 1.0;
    double maxDrivePowerStep = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.update();
//        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
//        robot = new Robot(RobotRoundhouse.getParamsAuto(ControlHubDeviceNameManager.getControlHubDeviceNameManager().getDeviceName(), hardwareMap), hardwareMap);
        robot = new Robot24(RobotRoundhouse.getParamsAuto(hardwareMap), hardwareMap);
        RRMecanumDrive drive = robot.drive;

//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

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
