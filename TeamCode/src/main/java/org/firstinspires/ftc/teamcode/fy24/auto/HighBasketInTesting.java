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

@Autonomous(name="DO NOT USE THIS HIGH BASKET TEST. I REPEAT DO NOT USE THIS OUTSIDE OF TESTING UNTIL FURTHER NOTICE.")
public class HighBasketInTesting extends LinearOpMode {
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

        FieldyTeleOpScheme23 controlScheme = new FieldyTeleOpScheme23(gamepad1, gamepad2, robot.imu);

        AutoSequenceSwitcher switcher = new AutoSequenceSwitcher();

        Pose2d startPose = new Pose2d(0, 0, 0);
        switcher.addSequence("HighBasket", robot.drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(14, -14))
                .turn(Math.toRadians(45))
                .forward(0.5)
                .addTemporalMarker(() -> {
//                    while ((double) ((armRightPivot.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerDegree < 75) {
//                        armRightPivot.setVelocity(armPivotSpeed);
//                        armLeftPivot.setVelocity(armPivotSpeed);
//                    }
                    armRightPivot.setTargetPosition(2200);
                    armLeftPivot.setTargetPosition(2200);
                    armRightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLeftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRightPivot.setPower(1);
                    armLeftPivot.setPower(1);
                    while(armLeftPivot.getCurrentPosition() < 2000) {
                        telemetry.addData("leftPivotPos", armLeftPivot.getCurrentPosition());
                        telemetry.update();
                    }
//                    armRightPivot.setVelocity(0);
//                    armLeftPivot.setVelocity(0);

                    armRightExtend.setTargetPosition(4600);
                    armLeftExtend.setTargetPosition(4600);
                    armRightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLeftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRightExtend.setPower(1);
                    armLeftExtend.setPower(1);
                    while (armLeftExtend.getCurrentPosition() < 4400) {
                        telemetry.addData("leftExtendPos", armLeftExtend.getCurrentPosition());
                        telemetry.update();
                    }

                    intakeServo.setPower(-1);
                    sleep(2500);
                    intakeServo.setPower(0);

                    armLeftExtend.setTargetPosition(0);
                    armRightExtend.setTargetPosition(0);

                    sleep(1000);

                    armRightPivot.setTargetPosition(0);
                    armLeftPivot.setTargetPosition(0);

                    armRightPivot.setPower(1);
                    armLeftPivot.setPower(1);

                    while(armLeftExtend.getCurrentPosition() > 100) {
                        telemetry.addData("leftExtendPos", armLeftExtend.getCurrentPosition());
                        telemetry.update();
                    }
                    while (armLeftPivot.getCurrentPosition() > 100) {
                        telemetry.addData("leftPivotPos", armLeftPivot.getCurrentPosition());
                        telemetry.update();
                    }
                })
                .back(0.5)
                .turn(Math.toRadians(-45))
                .back(13)
                .strafeRight(24)

                // Might need tuning

                .addTemporalMarker(() -> {
                    intakeServo.setPower(1);
                    armRightExtend.setTargetPosition(750);
                    armLeftExtend.setTargetPosition(750);
                    sleep(2000);
                    intakeServo.setPower(0);
                    armLeftExtend.setTargetPosition(0);
                    armRightExtend.setTargetPosition(0);
                })

                // Too far
                .strafeLeft(24)
                .forward(13)
                .turn(Math.toRadians(45))
                .strafeLeft(2)
                .forward(3.3)
                .addTemporalMarker(() -> {
//                    while ((double) ((armRightPivot.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerDegree < 75) {
//                        armRightPivot.setVelocity(armPivotSpeed);
//                        armLeftPivot.setVelocity(armPivotSpeed);
//                    }
                    armRightPivot.setTargetPosition(2150);
                    armLeftPivot.setTargetPosition(2150);
                    armRightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLeftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRightPivot.setPower(1);
                    armLeftPivot.setPower(1);
                    while(armLeftPivot.getCurrentPosition() < 2000) {
                        telemetry.addData("leftPivotPos", armLeftPivot.getCurrentPosition());
                        telemetry.update();
                    }
//                    armRightPivot.setVelocity(0);
//                    armLeftPivot.setVelocity(0);

                    armRightExtend.setTargetPosition(4600);
                    armLeftExtend.setTargetPosition(4600);
                    armRightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLeftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRightExtend.setPower(1);
                    armLeftExtend.setPower(1);
                    while (armLeftExtend.getCurrentPosition() < 4200) {
                        telemetry.addData("leftExtendPos", armLeftExtend.getCurrentPosition());
                        telemetry.update();
                    }
                    intakeServo.setPower(-1);
                    sleep(2500);
                    intakeServo.setPower(0);

                    armLeftExtend.setTargetPosition(0);
                    armRightExtend.setTargetPosition(0);

                    while(armLeftExtend.getCurrentPosition() > 100) {
                        telemetry.addData("leftExtendPos", armLeftExtend.getCurrentPosition());
                        telemetry.update();
                    }

                    armRightPivot.setTargetPosition(0);
                    armLeftPivot.setTargetPosition(0);

                    while (armLeftPivot.getCurrentPosition() > 100) {
                        telemetry.addData("leftPivotPos", armLeftPivot.getCurrentPosition());
                        telemetry.update();
                    }
                })
                .build()
        );


        telemetry.addLine("Ready.");
        telemetry.update();
        waitForStart();


        switcher.selectName("HighBasket");
        robot.drive.followTrajectorySequence(switcher.getSelected().getTrajectorySequence());
    }
}