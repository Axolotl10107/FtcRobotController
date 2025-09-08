package org.firstinspires.ftc.teamcode.teletest.framework;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;

@Disabled
@Autonomous(name="RRMecanumDrive Spline Test", group="TeleTest")
public class RRMDSpline extends OpMode {

    private Robot25 robot;

    @Override
    public void init() {
        try {
            robot = new Robot25(RobotRoundhouse25.getParamsAuto(hardwareMap), hardwareMap);
        } catch (Robot25.InvalidDeviceClassException | RobotRoundhouse25.OldRobotException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void start() {
        Telemetry.Item stage = telemetry.addData("Stage", "Initializing");
        telemetry.setAutoClear(false);
        telemetry.update();

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(90));
        robot.drive.setPoseEstimate(startPose);

        stage.setValue("Coordinates");
        telemetry.update();
        telemetry.update();
        TrajectorySequence trajSeq = robot.drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(10, 10), 0)
                .splineToConstantHeading(new Vector2d(10, 0), 0)
                .splineToConstantHeading(new Vector2d(0, 0), 0)
                .build();
        robot.drive.followTrajectorySequence(trajSeq);
//        robot.drive.turn(Math.toRadians(90));
//        robot.drive.followTrajectorySequence(trajSeq);

        telemetry.addData("Current Heading (RR)", robot.drive.getPoseEstimate().getHeading());
        telemetry.addData("Current Heading (IMU)", robot.imu.yaw());

        stage.setValue("Forward");
        telemetry.update();
        Trajectory forward = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .forward(10)
                .build();
        robot.drive.followTrajectory(forward);

        stage.setValue("Left");
        telemetry.update();
        Trajectory left = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .strafeLeft(10)
                .build();
        robot.drive.followTrajectory(left);

        stage.setValue("Backwards");
        telemetry.update();
        Trajectory backwards = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .back(10)
                .build();
        robot.drive.followTrajectory(backwards);

        stage.setValue("Right");
        telemetry.update();
        Trajectory right = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .strafeRight(10)
                .build();
        robot.drive.followTrajectory(right);

        stage.setValue("Coordinates with getPoseEstimate() instead of startPose");
        telemetry.update();
        TrajectorySequence trajSeq2 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(10, 10))
                .lineToConstantHeading(new Vector2d(10, 0))
                .lineToConstantHeading(new Vector2d(0, 0))
                .build();
        robot.drive.followTrajectorySequence(trajSeq2);

        requestOpModeStop();
    }

    @Override
    public void loop() {

    }
}
