package org.firstinspires.ftc.teamcode.fy24.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse24;

@Disabled
@Autonomous()
public class ParkFromNear extends OpMode {

    Robot24 robot;
    TrajectorySequence trajSeq;

    @Override
    public void init() {
        robot = new Robot24(RobotRoundhouse24.getParamsAuto(hardwareMap), hardwareMap);

        robot.drive.setPoseEstimate(new Pose2d(0, 0, 0));
        trajSeq = robot.drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .lineToConstantHeading(new Vector2d((24*2)+2, 0)) // +2 for some tolerance
                .build();
    }

    @Override
    public void start() {
        robot.drive.followTrajectorySequence(trajSeq);
    }

    @Override
    public void loop() {
        robot.update();
    }
}
