package org.firstinspires.ftc.teamcode.fy25.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.framework.adapters.DualDcMotorEx;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name="ThreeLeave")
public class ThreeLeave extends LinearOpMode {
    Robot25 robot;
    TrajectorySequence mainTrajectory;

    void unload() {
        robot.launchWheelSimple.spinUp(0.95);
        robot.update();
        sleep(800);
        robot.loader.load();
        robot.update();
        sleep(2000);
        robot.loader.pass();
        robot.launchWheelSimple.spinDown();
        robot.update();
        sleep(900);
        robot.indexer.next();
        int i = 0;
        while (i < 100) {
            i++;
            robot.update();
            sleep(5);
        }
        robot.update();
        sleep(250);
        robot.launchWheelSimple.spinUp(0.95);
        robot.update();
        sleep(800);
        robot.loader.load();
        robot.update();
        sleep(2000);
        robot.loader.pass();
        robot.launchWheelSimple.spinDown();
        robot.update();
        sleep(900);
        robot.indexer.next();
        i = 0;
        while (i < 100) {
            i++;
            robot.update();
            sleep(5);
        }
        robot.update();
        sleep(250);
        robot.launchWheelSimple.spinUp(0.95);
        robot.update();
        sleep(800);
        robot.loader.load();
        robot.update();
        sleep(2000);
        robot.loader.pass();
        robot.launchWheelSimple.spinDown();
        robot.update();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot25(RobotRoundhouse25.getRobotBParams(hardwareMap), hardwareMap);

        waitForStart();
        unload();
    }
}
