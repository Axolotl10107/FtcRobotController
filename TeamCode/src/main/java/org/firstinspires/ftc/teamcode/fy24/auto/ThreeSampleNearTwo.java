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
import org.firstinspires.ftc.teamcode.framework.util.autoSwitch.AutoSequenceSwitcher;
//import org.firstinspires.ftc.teamcode.fy23.controls.ctlpad.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse;

@Autonomous(name="3 Samples from Near", group="HighBasket")
public class ThreeSampleNearTwo extends LinearOpMode {
    Robot24 robot;
    Telemetry opModeTelemetry;

    private DcMotorEx armLeftExtend;
    private DcMotorEx armRightExtend;
    private DcMotorEx armLeftPivot;
    private DcMotorEx armRightPivot;

    private CRServo intakeServo;

    // For dynamic extension limit (which is currently commented out)
    double armExtendSpeed = 1110;
    double armPivotSpeed = 1115;


    final double ticksPerInch = 157.86;
    final double ticksPerDegree = 32.06;

    final int highPivotPos = 2300;
    final int highExtendPos = 4500;

    // for grabbing additional samples
    final int firstGrabExtendPos = 750;
    final int secondGrabExtendPos = 1400;

    final int lowPivotPos = 0;
    final int lowExtendPos = 0;

    // Utility methods
    void pivotToBasket() {
        armLeftPivot.setTargetPosition(highPivotPos);
        armRightPivot.setTargetPosition(highPivotPos);
        armLeftPivot.setPower(1);
        armRightPivot.setPower(1);
    }

    void waitForPivotToBasket() {
        while(armLeftPivot.getCurrentPosition() < (highPivotPos - 100)) {
//            telemetry.addLine("Pivot --> Basket");
//            telemetry.addData("leftPivotPos", armLeftPivot.getCurrentPosition());
//            telemetry.update();
        }
    }

    void extendToBasket() {
        armLeftExtend.setTargetPosition(highExtendPos);
        armRightExtend.setTargetPosition(highExtendPos);
    }

    void waitForExtendToBasket() {
        while (armLeftExtend.getCurrentPosition() < (highExtendPos - 300)) {
            telemetry.addLine("Extend --> Basket");
            telemetry.addData("leftExtendPos", armLeftExtend.getCurrentPosition());
            telemetry.update();
        }
    }

    void pivotToGround() {
        armLeftPivot.setTargetPosition(lowPivotPos);
        armRightPivot.setTargetPosition(lowPivotPos);
        armLeftPivot.setPower(0.7);
        armRightPivot.setPower(0.7);
    }

    void waitForPivotToGround() {
        while (armLeftPivot.getCurrentPosition() > (lowPivotPos + 100)) {
            telemetry.addLine("Pivot --> Ground");
            telemetry.addData("leftPivotPos", armLeftPivot.getCurrentPosition());
            telemetry.update();
        }
    }

    void extendToGround() {
        armLeftExtend.setTargetPosition(lowExtendPos);
        armRightExtend.setTargetPosition(lowExtendPos);
    }

    void waitForExtendToGround() {
        while(armLeftExtend.getCurrentPosition() > (lowExtendPos + 100)) {
            telemetry.addLine("Extend --> Ground");
            telemetry.addData("leftExtendPos", armLeftExtend.getCurrentPosition());
            telemetry.update();
        }
    }

    void extendToFirstGrab() {
        armLeftExtend.setTargetPosition(firstGrabExtendPos);
        armRightExtend.setTargetPosition(firstGrabExtendPos);
    }

    void extendToSecondGrab() {
        armLeftExtend.setTargetPosition(secondGrabExtendPos);
        armRightExtend.setTargetPosition(secondGrabExtendPos);
    }

    void sampleOut() {
        intakeServo.setPower(-1);
        sleep(2500);
        intakeServo.setPower(0);
    }

    void sampleIn() {
        intakeServo.setPower(1);
        sleep(2500);
        intakeServo.setPower(0);
    }

    void scoreSample() {
        // Make sure manipulator is in position
        waitForPivotToBasket();
        waitForExtendToBasket();

        // Score sample
        sampleOut();

        // Give extend a headstart so we don't hit the low basket
        extendToGround();
        sleep(1000);
        pivotToGround();
    }

    void brake() {
        robot.drive.getLeftFrontMotor().setTargetPosition(robot.drive.getLeftFrontMotor().getCurrentPosition());
        robot.drive.getRightFrontMotor().setTargetPosition(robot.drive.getRightFrontMotor().getCurrentPosition());
        robot.drive.getLeftBackMotor().setTargetPosition(robot.drive.getLeftBackMotor().getCurrentPosition());
        robot.drive.getRightBackMotor().setTargetPosition(robot.drive.getRightBackMotor().getCurrentPosition());
    }

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

        opModeTelemetry = telemetry;

        robot = new Robot24(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);

//        FieldyTeleOpScheme23 controlScheme = new FieldyTeleOpScheme23(gamepad1, gamepad2, robot.imu);

        AutoSequenceSwitcher switcher = new AutoSequenceSwitcher();

        armLeftPivot.setTargetPosition(armLeftPivot.getCurrentPosition());
        armRightPivot.setTargetPosition(armRightPivot.getCurrentPosition());
        armLeftExtend.setTargetPosition(armLeftExtend.getCurrentPosition());
        armRightExtend.setTargetPosition(armRightExtend.getCurrentPosition());
        armLeftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeftPivot.setPower(1);
        armRightPivot.setPower(1);
        armLeftExtend.setPower(1);
        armRightExtend.setPower(1);

        Pose2d startPose = new Pose2d(0, 0, 0);
        robot.drive.setPoseEstimate(startPose);
        TrajectorySequence toBasket1 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(15, -13, Math.toRadians(45)))
                .forward(2)
                .build();

        TrajectorySequence toBasket2 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .back(2)
                .lineToLinearHeading(new Pose2d(17, -8, Math.toRadians(45)))
                .build();

        TrajectorySequence toBasket3 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(17, -3, Math.toRadians(45)))
                .build();

        TrajectorySequence toFirstGrab = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-3, -16, 0))
                .lineToConstantHeading(new Vector2d(-3, -37))
                .build();

        TrajectorySequence toSecondGrab = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(10, -34, Math.toRadians(0)))
                .build();


        telemetry.addLine("Ready.");
        telemetry.update();
        waitForStart();


// ------------------------ First Sample ----------------------------------
        // Start manipulator moving
        pivotToBasket();
        extendToBasket();

        // Drive to basket while manipulator moves
        robot.drive.followTrajectorySequence(toBasket1);
        brake();

        // Make sure manipulator is in position
        waitForPivotToBasket();
        waitForExtendToBasket();

        // Score first sample
        scoreSample();


// ------------------------ Second Sample ----------------------------------
        // Go to grab second sample
        robot.drive.followTrajectorySequence(toFirstGrab);

        // Grab sample
        extendToFirstGrab();
        sampleIn();

        // Move to basket
        pivotToBasket();
        extendToBasket();
        robot.drive.followTrajectorySequence(toBasket2);
        brake();

        // Make sure manipulator is in position
        waitForPivotToBasket();
        waitForExtendToBasket();

        // Score second sample
        scoreSample();


// ------------------------ Third Sample ----------------------------------
        // Go to grab third sample
        robot.drive.followTrajectorySequence(toSecondGrab);

        // Grab sample
        extendToFirstGrab();
        sampleIn();

        // Move to basket
        pivotToBasket();
        extendToBasket();
        robot.drive.followTrajectorySequence(toBasket3);
        brake();

        // Make sure manipulator is in position
        waitForPivotToBasket();
        waitForExtendToBasket();

        // Score third sample
        scoreSample();

        // Try to get back to the ground before the OpMode ends
        waitForPivotToGround();
        waitForExtendToGround();
    }
}