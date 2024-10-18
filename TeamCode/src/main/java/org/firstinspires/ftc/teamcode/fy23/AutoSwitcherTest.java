package org.firstinspires.ftc.teamcode.fy23;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fy23.autoSwitch.AutoSequenceSwitcher;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.FieldyTeleOpScheme;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.TeleOpState;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;

@Autonomous()
public class AutoSwitcherTest extends LinearOpMode {

    Robot robot;
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

        robot = new Robot(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);

        FieldyTeleOpScheme controlScheme = new FieldyTeleOpScheme(gamepad1, gamepad2, robot.imu);


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
