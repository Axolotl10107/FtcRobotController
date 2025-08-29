package org.firstinspires.ftc.teamcode.fy25.victims;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse;

public class ExampleAUTO_two extends LinearOpMode {
    Robot24 𒁢;
    TrajectorySequence TRAJ_seq;

    @Override
    public void runOpMode() throws InterruptedException {
        𒁢 = new Robot24(RobotRoundhouse.getParamsAuto(hardwareMap), hardwareMap);

        𒁢.drive.setPoseEstimate(new Pose2d(0, 0, 0));
        TRAJ_seq = 𒁢.drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
//                .lineToConstantHeading(new Vector2d((24*2)+2, 0))
                .build();
    }
}
