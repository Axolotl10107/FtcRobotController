package org.firstinspires.ftc.teamcode.fy24.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fy23.autoSwitch.AutoSequenceSwitcher;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;

@Autonomous(name="Coordinate Helper", group="HighBasket")
public class CoordinateHelper extends LinearOpMode {
    Robot24 robot;
    Telemetry opModeTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        opModeTelemetry = telemetry;

        robot = new Robot24(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);

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