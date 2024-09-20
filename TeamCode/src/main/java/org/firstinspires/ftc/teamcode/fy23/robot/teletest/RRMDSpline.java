package org.firstinspires.ftc.teamcode.fy23.robot.teletest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;

@Autonomous(group="TeleTest")
public class RRMDSpline extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
    }

    @Override
    public void start() {
        Pose2d startPose = new Pose2d(0,0,0);
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = robot.drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(10, 10))
                .lineToConstantHeading(new Vector2d(10, 0))
                .lineToConstantHeading(new Vector2d(0, 0))
                .build();

        robot.drive.followTrajectorySequence(trajSeq);
        robot.drive.turn(Math.toRadians(90));
        robot.drive.followTrajectorySequence(trajSeq);

        requestOpModeStop();
    }

    @Override
    public void loop() {

    }
}
