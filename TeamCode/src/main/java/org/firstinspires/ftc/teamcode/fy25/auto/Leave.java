package org.firstinspires.ftc.teamcode.fy25.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Leave")
public class Leave extends LinearOpMode {

    Robot25 robot;
    TrajectorySequence mainTrajSeq;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = RobotRoundhouse25.getRobotAuto(hardwareMap);
        } catch (RobotRoundhouse25.OldRobotException e) {
            throw new RuntimeException(e);
        }

        robot.drive.setPoseEstimate(new Pose2d(0, 0, 0));
        mainTrajSeq = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineTo(new Vector2d(40, 0))
                .build();

        waitForStart();

        robot.drive.followTrajectorySequence(mainTrajSeq);
    }
}
