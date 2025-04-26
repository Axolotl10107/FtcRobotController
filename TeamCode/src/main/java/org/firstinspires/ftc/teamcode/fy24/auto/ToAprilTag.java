package org.firstinspires.ftc.teamcode.fy24.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse;

@Autonomous(name = "ToAprilTag")
public class ToAprilTag extends LinearOpMode {

    Robot24 robot;
    AprilTagUtils aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot
        robot = new Robot24(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);

        // Initialize AprilTagUtils, which internally initializes the AprilTagProcessor and VisionPortal
        aprilTag = new AprilTagUtils(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);
        robot.drive.setPoseEstimate(startPose);

        // Create a trajectory that moves the robot forward by the distance detected by the AprilTag
        TrajectorySequence toAprilTag = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .forward(aprilTag.getDistance() - 2)  // Adjusting distance based on detected AprilTag
                .build();

        waitForStart();

        // Follow the trajectory once the op mode starts
        robot.drive.followTrajectorySequence(toAprilTag);

        // Clean up vision system
        aprilTag.stopVision();
    }
}
