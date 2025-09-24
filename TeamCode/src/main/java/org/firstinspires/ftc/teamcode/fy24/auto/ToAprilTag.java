package org.firstinspires.ftc.teamcode.fy24.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse24;

@Autonomous(name = "ToAprilTag")
public class ToAprilTag extends LinearOpMode {

    Robot24 robot;
    AprilTagUtils aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot
        robot = new Robot24(RobotRoundhouse24.getRobotAParams(hardwareMap), hardwareMap);

        CRServo cameraServo;

        cameraServo = hardwareMap.get(CRServo.class, "cameraServo");

        // Initialize AprilTagUtils, which internally initializes the AprilTagProcessor and VisionPortal
        aprilTag = new AprilTagUtils(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);
        robot.drive.setPoseEstimate(startPose);

        waitForStart();

        while (aprilTag.getDistanceY() == -1) {
            cameraServo.setPower(1);
            int i = 0;
            cameraServo.setPower(1);
            while (aprilTag.getDistanceY() == -1) {
                i += 1;
                if (i >= 30) {
                    break;
                }
                wait(100);
            }
            if (aprilTag.getDistanceY() == -1) {
                break;
            }
            cameraServo.setPower(-1);
            while (aprilTag.getDistanceY() == -1) {
                i += 1;
                if (i >= 30) {
                    break;
                }
                wait(100);
            }
            if (aprilTag.getDistanceY() == -1) {
                break;
            }
        }

        double angle = Math.tanh(aprilTag.getDistanceY() / aprilTag.getDistanceX());

        TrajectorySequence toAprilTag = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .turn(angle)
                .forward(aprilTag.getDistanceY() - 5)  // Adjusting distance based on detected AprilTag
                .build();
        // Follow the trajectory once the op mode starts
        robot.drive.followTrajectorySequence(toAprilTag);

        while (opModeIsActive()) {
            while (aprilTag.getDistanceY() == -1) {
                cameraServo.setPower(1);
                int i = 0;
                cameraServo.setPower(1);
                while (aprilTag.getDistanceY() == -1) {
                    i += 1;
                    if (i >= 30) {
                        break;
                    }
                    wait(100);
                }
                if (aprilTag.getDistanceY() == -1) {
                    break;
                }
                cameraServo.setPower(-1);
                while (aprilTag.getDistanceY() == -1) {
                    i += 1;
                    if (i >= 30) {
                        break;
                    }
                    wait(100);
                }
                if (aprilTag.getDistanceY() == -1) {
                    break;
                }
            }

            angle = Math.tanh(aprilTag.getDistanceY() / aprilTag.getDistanceX());

            toAprilTag = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .turn(angle)
                    .forward(aprilTag.getDistanceY() - 5)  // Adjusting distance based on detected AprilTag
                    .build();

            robot.drive.followTrajectorySequence(toAprilTag);
        }

        // Clean up vision system
        aprilTag.stopVision();
    }
}
