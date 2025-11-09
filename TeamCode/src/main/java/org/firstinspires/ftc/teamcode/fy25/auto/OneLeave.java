package org.firstinspires.ftc.teamcode.fy25.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheel;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Vector;

@Autonomous(name = "OneLeave")
public class OneLeave extends LinearOpMode {

    Robot25 robot;
    TrajectorySequence mainTrajSeq;

    void spinUp() {
        robot.launchWheelBack.spinUp();
        robot.launchWheelFront.spinUp();
        robot.launchWheelBack.update();
        robot.launchWheelFront.update();
    }

    void spinDown() {
        robot.launchWheelBack.spinDown();
        robot.launchWheelFront.spinDown();
        robot.launchWheelBack.update();
        robot.launchWheelFront.update();
    }

    void score() {
        spinUp();
        sleep(1000);
        robot.launchGate.open();
        sleep(500);
        robot.launchGate.close();
    }

    void scoreSecond() {
        robot.motorIntake.spinIn();
        spinUp();
        sleep(1000);
        robot.launchGate.open();
        robot.motorIntake.spinIn();
        sleep(1000);
        robot.launchGate.close();
        robot.motorIntake.stop();
        spinDown();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new Robot25(RobotRoundhouse25.getRobotAParams(hardwareMap), hardwareMap);
        } catch (Robot25.InvalidDeviceClassException e) {
            throw new RuntimeException(e);
        }

        robot.drive.setPoseEstimate(new Pose2d(0, 0, 0));
        mainTrajSeq = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineTo(new Vector2d(15, -10))
                .addTemporalMarker(2, () -> {
                    score();
                    scoreSecond();
                })
                .waitSeconds(3)
                .lineTo(new Vector2d(20, -30))
                .build();

        waitForStart();

        robot.drive.followTrajectorySequence(mainTrajSeq);
    }
}
