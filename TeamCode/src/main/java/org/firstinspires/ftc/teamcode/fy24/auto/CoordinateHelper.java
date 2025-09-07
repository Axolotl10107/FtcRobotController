package org.firstinspires.ftc.teamcode.fy24.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.util.autoSwitch.AutoSequenceSwitcher;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse24;

@Autonomous(name="Coordinate Helper", group="HighBasket")
public class CoordinateHelper extends LinearOpMode {
    Robot24 robot;
    Telemetry opModeTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        opModeTelemetry = telemetry;

        robot = new Robot24(RobotRoundhouse24.getRobotAParams(hardwareMap), hardwareMap);

        AutoSequenceSwitcher switcher = new AutoSequenceSwitcher();

        Pose2d startPose = new Pose2d(0, 0, 0);
        switcher.addSequence("HighBasket", robot.drive.trajectorySequenceBuilder(startPose)
                        .lineToConstantHeading(new Vector2d(10, 0))
                        .lineToConstantHeading(new Vector2d(0, 0))
                        .lineToConstantHeading(new Vector2d(-10, 0))
                        .lineToConstantHeading(new Vector2d(0, 0))
                        .lineToConstantHeading(new Vector2d(0, 10))
                        .lineToConstantHeading(new Vector2d(0, 0))
                        .lineToConstantHeading(new Vector2d(0, -10))
                        .lineToConstantHeading(new Vector2d(0, 0))
                .build()
        );


        telemetry.addLine("Ready.");
        telemetry.update();
        waitForStart();


        switcher.selectName("HighBasket");
        robot.drive.followTrajectorySequence(switcher.getSelected().getTrajectorySequence());
    }
}