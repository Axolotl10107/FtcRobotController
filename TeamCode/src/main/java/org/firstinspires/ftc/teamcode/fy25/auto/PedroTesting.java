package org.firstinspires.ftc.teamcode.fy25.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;

@Autonomous(name = "PedroTesting")
public class PedroTesting extends LinearOpMode {

    Robot25 robot;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(81, 9, 0);
    private final Pose scorePose = new Pose(15, 72, Math.toRadians(30));
    private final Pose firstPickupPose = new Pose(36, 96, Math.toRadians(90));

    public static class Paths {
        public static PathChain Path1;
        public static PathChain Path3;
        public static PathChain Path4;
        public static PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(60.822, 31.941),
                                    new Pose(35.550, 35.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35.550, 35.700),

                                    new Pose(30.675, 35.700)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30.675, 35.700),

                                    new Pose(20.166, 35.700)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(20.166, 35.700),
                                    new Pose(9.701, 37.189),
                                    new Pose(0.749, 18.420),
                                    new Pose(51.201, 14.142)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))

                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                unload();
                robot.motorIntake.spinIn();
                robot.motorIndexer.prepIntake();
                updateIndexer();
                follower.followPath(Paths.Path1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(Paths.Path3,true);
                    setPathState(2);
                    robot.indexer.intake();
                    updateIndexer();
                    robot.indexer.next();
                    robot.indexer.prepIntake();
                    updateIndexer();
                    robot.motorIntake.stop();
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(Paths.Path4,true);
                    setPathState(3);
                    robot.indexer.intake();
                    updateIndexer();
                    robot.indexer.next();
                    robot.indexer.prepIntake();
                    updateIndexer();
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(Paths.Path5,true);
                    robot.indexer.intake();
                    updateIndexer();
                    robot.indexer.next();
                    robot.indexer.prepIntake();
                    updateIndexer();
                    unload();
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    void updateIndexer() {
        int i = 0;
        while (i < 250) {
            i += 1;
            robot.update();
            sleep(1);
        }
    }

    void unload() {
        robot.launchWheel.spinUp();
        sleep(2000);
        robot.loader.load();
        sleep(250);
        robot.loader.pass();
        sleep(250);
        robot.motorIndexer.next();
        updateIndexer();
        sleep(250);
        robot.loader.load();
        sleep(250);
        robot.loader.pass();
        sleep(250);
        robot.motorIndexer.next();
        updateIndexer();
        sleep(250);
        robot.loader.load();
        sleep(250);
        robot.loader.pass();
        robot.launchWheel.spinDown();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = RobotRoundhouse25.getRobotAuto(hardwareMap);
        } catch (RobotRoundhouse25.OldRobotException e) {
            throw new RuntimeException(e);
        }

        waitForStart();

        autonomousPathUpdate();
    }
}
