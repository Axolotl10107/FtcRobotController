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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fy23.autoSwitch.AutoSequenceSwitcher;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.FieldyTeleOpScheme;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;

@Autonomous(name="HighBasket")
public class HighBasket extends LinearOpMode {
    Robot24 robot;
    Telemetry opModeTelemetry;

    private DcMotorEx armLeftExtend;
    private DcMotorEx armRightExtend;
    private DcMotorEx armLeftPivot;
    private DcMotorEx armRightPivot;

    private CRServo intakeServo;

    @Override
    public void runOpMode() throws InterruptedException {

        armLeftExtend = hardwareMap.get(DcMotorEx.class, "armLeftExtend");
        armRightExtend = hardwareMap.get(DcMotorEx.class, "armRightExtend");
        armLeftPivot = hardwareMap.get(DcMotorEx.class, "armLeftPivot");
        armRightPivot = hardwareMap.get(DcMotorEx.class, "armRightPivot");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        armLeftExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        armRightExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armRightPivot.setDirection(DcMotorSimple.Direction.FORWARD);

        armLeftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double armExtendSpeed = 1110;
        double armPivotSpeed = 1115;

        final double ticksPerInch = 157.86;
        final double ticksPerDegree = 32.06;

        opModeTelemetry = telemetry;

        robot = new Robot24(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);

        FieldyTeleOpScheme controlScheme = new FieldyTeleOpScheme(gamepad1, gamepad2, robot.imu);

        AutoSequenceSwitcher switcher = new AutoSequenceSwitcher();

        Pose2d startPose = new Pose2d(0, 0, 0);
        switcher.addSequence("HighBasket", robot.drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(14, -14))
                .turn(Math.toRadians(45))
                .addTemporalMarker(() -> {
//                    while ((double) ((armRightPivot.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerDegree < 75) {
//                        armRightPivot.setVelocity(armPivotSpeed);
//                        armLeftPivot.setVelocity(armPivotSpeed);
//                    }
                    armRightPivot.setTargetPosition(2300);
                    armLeftPivot.setTargetPosition(2300);
                    armRightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLeftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRightPivot.setPower(0.5);
                    armLeftPivot.setPower(0.5);
                    while(armLeftPivot.getCurrentPosition() < 2200) {
                        telemetry.addData("leftPivotPos", armLeftPivot.getCurrentPosition());
                        telemetry.update();
                    }
//                    armRightPivot.setVelocity(0);
//                    armLeftPivot.setVelocity(0);

                    armRightExtend.setTargetPosition(4600);
                    armLeftExtend.setTargetPosition(4600);
                    armRightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLeftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRightExtend.setPower(0.5);
                    armLeftExtend.setPower(0.5);
                    while (armLeftExtend.getCurrentPosition() < 4500) {
                        telemetry.addData("leftExtendPos", armLeftExtend.getCurrentPosition());
                        telemetry.update();
                    }

                    sleep(1000);
                    intakeServo.setPower(-1);
                    sleep(5000);
                    intakeServo.setPower(0);

                    armRightPivot.setTargetPosition(armRightExtend.getCurrentPosition() + 50);
                    armLeftPivot.setTargetPosition(armLeftExtend.getCurrentPosition() + 50);

                    armLeftExtend.setTargetPosition(50);
                    armRightExtend.setTargetPosition(50);
                    while(armLeftExtend.getCurrentPosition() > 100) {
                        telemetry.addData("leftExtendPos", armLeftExtend.getCurrentPosition());
                        telemetry.update();
                    }
                    armRightPivot.setTargetPosition(50);
                    armLeftPivot.setTargetPosition(50);
                    while (armLeftPivot.getCurrentPosition() > 100) {
                        telemetry.addData("leftPivotPos", armLeftPivot.getCurrentPosition());
                        telemetry.update();
                    }
                })
                .splineToConstantHeading(new Vector2d(0, 0), 0)
                .build()
        );

        switcher.selectName("HighBasket");
        robot.drive.followTrajectorySequence(switcher.getSelected().getTrajectorySequence());
    }
}