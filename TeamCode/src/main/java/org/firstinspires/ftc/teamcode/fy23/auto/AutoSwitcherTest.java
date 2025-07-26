package org.firstinspires.ftc.teamcode.fy23.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.util.autoSwitch.AutoSequenceSwitcher;
import org.firstinspires.ftc.teamcode.framework.ctlpad.teleop.fy23.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse;

@Autonomous()
@Disabled
public class AutoSwitcherTest extends LinearOpMode {

    Robot24 robot;
    double telemetryPower;
    Telemetry opModeTelemetry;

    private void updateTelemetry() {
        Telemetry telemetry = new MultipleTelemetry(opModeTelemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Set power", telemetryPower);
        telemetry.addData("Actual Power", robot.arm.getPivotPower());
        telemetry.addData("Actual Velocity", robot.arm.getPivotVelocity());
        telemetry.addData("Actual Position", robot.arm.getPivotPosition());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        opModeTelemetry = telemetry;

        robot = new Robot24(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);

        FieldyTeleOpScheme23 controlScheme = new FieldyTeleOpScheme23(gamepad1, gamepad2, robot.imu);


        waitForStart();

        AutoSequenceSwitcher switcher = new AutoSequenceSwitcher();

        Pose2d startPose = new Pose2d(0, 0, 0);

        switcher.addSequence("1", robot.drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10, 10), 0)
                .build()
        );

        switcher.addSequence("2", robot.drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-10, -10), 0)
                .build()
        );

        switcher.selectFirst();


        boolean chosen = false;
        while(!chosen) {
            if (gamepad1.dpad_up) {
                switcher.selectNext();
            } else if (gamepad1.dpad_down) {
                switcher.selectPrevious();
            } else if (gamepad1.a) {
                break;
            }

            telemetry.addData("current automode", switcher.getSelected().getName());
            telemetry.update();
        }
    }

}
