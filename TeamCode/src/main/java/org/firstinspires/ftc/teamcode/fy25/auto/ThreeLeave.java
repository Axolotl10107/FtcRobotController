package org.firstinspires.ftc.teamcode.fy25.auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.framework.adapters.DualDcMotorEx;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name="ThreeLeave")
public class ThreeLeave extends LinearOpMode {
    Robot25 robot;
    TrajectorySequence mainTrajectory;

    void unload() {
        robot.launchWheelSimple.spinUp(5400);
        robot.launchWheelSimple.update();
        sleep(4500);
        robot.loader.load();
        robot.loader.update();
        sleep(3500);
//        robot.indexer.unload();
//        robot.update();
//        sleep(4000);
        robot.loader.pass();
        robot.loader.update();
        sleep(800);
        robot.loader.load();
        robot.loader.update();
        sleep(2000);
        robot.loader.pass();
        robot.loader.update();
        sleep(900);
        robot.motorIndexer.next();
        int i = 0;
        while (i < 100) {
            i++;
            robot.motorIndexer.update();
            sleep(5);
        }
        sleep(800);
        robot.loader.load();
        robot.loader.update();
        sleep(2000);
        robot.loader.pass();
        robot.loader.update();
        sleep(900);
        robot.motorIndexer.next();
        i = 0;
        while (i < 100) {
            i++;
            robot.motorIndexer.update();
            sleep(5);
        }
        sleep(800);
        robot.loader.load();
        robot.loader.update();
        sleep(2000);
        robot.loader.pass();
        robot.loader.update();
        robot.launchWheelSimple.spinDown();
        robot.launchWheelSimple.update();
        sleep(2000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot25(RobotRoundhouse25.getRobotBParams(hardwareMap), hardwareMap);

        mainTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineTo(new Vector2d(-25, -25))
                .build();

        waitForStart();
        unload();
        robot.drive.followTrajectorySequence(mainTrajectory);
    }
}
