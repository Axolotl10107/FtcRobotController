package org.firstinspires.ftc.teamcode.fy24.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.util.autoSwitch.AutoSequenceSwitcher;
//import org.firstinspires.ftc.teamcode.fy23.controls.ctlpad.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse24;

@Autonomous(name="3 Samples from Near Claw", group="HighBasket")
public class FourSampleNearClaw extends LinearOpMode {
    Robot24 robot;
    Telemetry opModeTelemetry;

    private DcMotorEx armLeftExtend;
    private DcMotorEx armRightExtend;
    private DcMotorEx armLeftPivot;
    private DcMotorEx armRightPivot;

    private Servo clawServo;
    private Servo clawPivotServo;

    // For dynamic extension limit (which is currently commented out)
    double armExtendSpeed = 1110;
    double armPivotSpeed = 1115;


    final int ticksPerInch = 158;
    final int ticksPerDegree = 32;

    final int highishPivotPos = 2200;
    final int highPivotPos = 2500;
    final int higherPivotPos = 2800;
    final int highExtendPos = 4600;

    // for grabbing additional samples
    final int firstGrabExtendPos = 3500;
    final int secondGrabExtendPos = 3700;
    final int thirdGrabExtendPos = 3800;

    final int lowPivotPos = 0;
    final int lowExtendPos = 0;

    // Utility methods
    void pivotToBasket() {
        armLeftPivot.setTargetPosition(highPivotPos);
        armRightPivot.setTargetPosition(highPivotPos);
        armLeftPivot.setPower(1);
        armRightPivot.setPower(1);
    }

    void pivotHighToBasket() {
        armLeftPivot.setTargetPosition(higherPivotPos);
        armRightPivot.setTargetPosition(higherPivotPos);
        armLeftPivot.setPower(1);
        armRightPivot.setPower(1);
    }

    void pivotHighishToBasket() {
        armLeftPivot.setTargetPosition(highishPivotPos);
        armRightPivot.setTargetPosition(highishPivotPos);
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

    void waitForExtendToFirstGrab() {
        while(armLeftExtend.getCurrentPosition() < (firstGrabExtendPos - 100)) {
            telemetry.addLine("Extend --> First grab");
            telemetry.addData("LeftExtendPos", armLeftExtend.getCurrentPosition());
            telemetry.update();
        }
    }

    void extendToSecondGrab() {
        armLeftExtend.setTargetPosition(secondGrabExtendPos);
        armRightExtend.setTargetPosition(secondGrabExtendPos);
    }

    void waitForExtendToSecondGrab() {
        while(armLeftExtend.getCurrentPosition() < (secondGrabExtendPos - 300)) {
            telemetry.addLine("Extend --> Second grab");
            telemetry.addData("LeftExtendPos", armLeftExtend.getCurrentPosition());
            telemetry.update();
        }
    }

    void extendToThirdGrab() {
        armLeftExtend.setTargetPosition(thirdGrabExtendPos);
        armRightExtend.setTargetPosition(thirdGrabExtendPos);
    }

    void waitForExtendToThirdGrab() {
        while(armLeftExtend.getCurrentPosition() < (thirdGrabExtendPos - 300)) {
            telemetry.addLine("Extend --> Third grab");
            telemetry.addData("LeftExtendPos", armLeftExtend.getCurrentPosition());
            telemetry.update();
        }
    }

    void sampleOut() {
        clawPivotServo.setPosition(0.7);
        sleep(300);
        clawServo.setPosition(0);
    }

    void exitBasket() {
        clawPivotServo.setPosition(1);
        armRightPivot.setTargetPosition(3000);
        armLeftPivot.setTargetPosition(3000);
    }

    void sampleIn() {
        clawPivotServo.setPosition(0.7);
        sleep(300);
        clawServo.setPosition(0.5);
    }

    void scoreSample() {
        // Make sure manipulator is in position
        waitForPivotToBasket();
        waitForExtendToBasket();

        // Score sample
        sampleOut();
        sleep(1000);
        clawPivotServo.setPosition(1);
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
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawPivotServo = hardwareMap.get(Servo.class, "clawPivotServo");

        armLeftExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        armRightExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armRightPivot.setDirection(DcMotorSimple.Direction.FORWARD);

        armLeftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        opModeTelemetry = telemetry;

        robot = new Robot24(RobotRoundhouse24.getRobotAParams(hardwareMap), hardwareMap);

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

        clawServo.setPosition(0.3);

        Pose2d startPose = new Pose2d(0, 0, 0);
        robot.drive.setPoseEstimate(startPose);
        TrajectorySequence toBasket1 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(15, -13, Math.toRadians(50)))
                .addTemporalMarker(4, () -> {
                    clawPivotServo.setPosition(1);
                    sleep(500);
                })
                .forward(8)
                .build();

        TrajectorySequence toBasket2 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .turn(Math.toRadians(50))
                .build();

        TrajectorySequence toBasket3 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .turn(Math.toRadians(42))
                .build();

        TrajectorySequence toBasket4 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .turn(Math.toRadians(30))
                .waitSeconds(2)
                .forward(4)
                .build();

        TrajectorySequence toFirstGrab = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .turn(Math.toRadians(-112))
                .build();

        TrajectorySequence toSecondGrab = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .turn(Math.toRadians(-89))
                .build();

        TrajectorySequence toThirdGrab = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .turn(Math.toRadians(-61))
                        .build();


        telemetry.addLine("Ready.");
        telemetry.update();
        waitForStart();


// ------------------------ First Sample ----------------------------------
        // Start manipulator moving
        pivotToBasket();
        extendToBasket();

        // Drive to basket while manipulator move
        robot.drive.followTrajectorySequence(toBasket1);
        brake();

        // Make sure manipulator is in position
        waitForPivotToBasket();
        waitForExtendToBasket();

        sleep(500);

        // Score first sample
        scoreSample();

        sleep(1000);

// ------------------------ Second Sample ----------------------------------
        robot.drive.followTrajectorySequence(toFirstGrab);
        extendToFirstGrab();
        sleep(500);
        pivotToGround();
        sleep(1000);
        sampleIn();
        sleep(1000);

        pivotToBasket();
        extendToBasket();

        waitForExtendToBasket();
        waitForPivotToBasket();

        robot.drive.followTrajectorySequence(toBasket2);

        sleep(500);

        scoreSample();

        //Rotate

        pivotToBasket();

        sleep(500);

        robot.drive.followTrajectorySequence(toSecondGrab);

        extendToSecondGrab();

        pivotToGround();

        sleep(3000);

        sampleIn();

        sleep(1000);

        pivotHighishToBasket();
        extendToBasket();

        waitForPivotToBasket();

        robot.drive.followTrajectorySequence(toBasket3);

        scoreSample();

        sleep(5000);

//
//
//        // Grab sample
//        extendToFirstGrab();
//
//        waitForExtendToFirstGrab();
//
//        sampleIn();
//
//        sleep(250);
//
//        // Move to basket
//        pivotToBasketHigh();
//        waitForPivotToBasket();
//        extendToBasket();
//        robot.drive.followTrajectorySequence(toBasket2);
//        brake();
//
//        // Make sure manipulator is in position
//        waitForPivotToBasket();
//        waitForExtendToBasket();
//
//        // Score second sample
//        scoreSample();


// ------------------------ Third Sample ----------------------------------

//        exitBasket();
//        sleep(750);
//        // Go to grab third sample
//        robot.drive.followTrajectorySequence(toSecondGrab);
//        sleep(500);
//        pivotToGround();
//
//        sleep(250);
//
//        // Grab sample
//        extendToSecondGrab();
//
//        waitForExtendToSecondGrab();
//
//        sampleIn();
//
//        sleep(250);
//
//        // Move to basket
//        pivotToBasketHigh();
//        waitForPivotToBasket();
//        extendToBasket();
//        robot.drive.followTrajectorySequence(toBasket3);
//        brake();
//
//        // Make sure manipulator is in position
//        waitForPivotToBasket();
//        waitForExtendToBasket();
//
//        // Score third sample
//        scoreSample();

// ------------------------ Fourth Sample ----------------------------------
//
//        exitBasket();
//        sleep(750);
//        // Go to fourth sample
//        robot.drive.followTrajectorySequence(toThirdGrab);
//        sleep(500);
//        pivotToGround();
//
//        sleep(250);
//
//        // Grab sample
//        extendToThirdGrab();
//
//        waitForExtendToThirdGrab();
//
//        sampleIn();
//
//        sleep(250);
//
////        Move to basket
//        pivotToBasketHigh();
//        waitForPivotToBasket();
//        extendToBasket();
//        robot.drive.followTrajectorySequence(toBasket4);
//        brake();
//
//        // Make sure manipulator is in position
//        waitForPivotToBasket();
//        waitForExtendToBasket();
//
//        // Score fourth sample
//        scoreSample();
//
//        // Try to get back to the ground before the OpMode ends
//        waitForPivotToGround();
//        waitForExtendToGround();
    }
}