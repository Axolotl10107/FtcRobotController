package org.firstinspires.ftc.teamcode.fy24.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.util.autoSwitch.AutoSequenceSwitcher;
//import org.firstinspires.ftc.teamcode.fy23.controls.ctlpad.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse24;

@Disabled
@Autonomous(name="BROKEN 3 Samples from Near", group="HighBasket")
public class ThreeSampleNear extends LinearOpMode {
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

    final int highPivotPos = 2200;
    final int highExtendPos = 4600;

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
        while (armLeftExtend.getCurrentPosition() < (highExtendPos - 100)) {
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
        sleep(500);
        pivotToGround();
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

        robot = new Robot24(RobotRoundhouse24.getRobotAParams(hardwareMap), hardwareMap);

//        FieldyTeleOpScheme23 controlScheme = new FieldyTeleOpScheme23(gamepad1, gamepad2, robot.imu);

        AutoSequenceSwitcher switcher = new AutoSequenceSwitcher();

        Pose2d startPose = new Pose2d(0, 0, 0);
        switcher.addSequence("HighBasket", robot.drive.trajectorySequenceBuilder(startPose)

                // Init. manipulator motors
                .addTemporalMarker(() -> {
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
                })

// ----------------------- First Sample -------------------------------------

                // Start getting manipulator into place
                .addTemporalMarker(() -> {
                    pivotToBasket();
                    extendToBasket();
                })

                // Drive to basket
                .lineToLinearHeading(new Pose2d(13, -13, Math.toRadians(45)))
//                .turn(Math.toRadians(45))
//                .forward(0.5)

                // Temporal marker with no offset runs in sequence (after the previous drive command)
                .addTemporalMarker(() -> {
                    // Score first sample
                    scoreSample();
                })

// ----------------------- Second Sample -------------------------------------

                // Back off a bit
//                .back(0.5)

                // Drive over to pick up another sample
                .lineToLinearHeading(new Pose2d(-3, -16, 0))
                .lineToConstantHeading(new Vector2d(-3, -37))
//                .lineToLinearHeading(new Pose2d(0, -38, Math.toRadians(0)))
//                .back(14)
//                .strafeRight(24)

                .addTemporalMarker(() -> {
                    // Grab first additional sample
                    extendToFirstGrab();
                    sampleIn(); // blocking!
//                    extendToGround(); // will run after sampleIn() has slept
                })

                // Start moving manipulator to basket position
                .addTemporalMarker(() -> {
                    pivotToBasket();
                    extendToBasket();
                })

                // Too far
//                .strafeLeft(24)
//                .forward(13)
//                .turn(Math.toRadians(45))
//                .strafeLeft(2)
//                .forward(3.3)

                // Drive back over to the basket
                // First one is a dummy move that gets cancelled for some reason.
                // Really! No idea why. I sure didn't cancel it.
//                .lineToLinearHeading(new Pose2d(10, -12.1, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    telemetry.addData("Hi", "There");
                    telemetry.update();
                })
                .lineToLinearHeading(new Pose2d(10, -12, Math.toRadians(45)))
//                .strafeLeft(26)
//                .forward(16.3)
//                .turn(Math.toRadians(45))

                .addTemporalMarker(() -> {
                    // Score second sample
                    scoreSample();
                })

// ----------------------- Third Sample -------------------------------------

                // Back off a bit
//                .back(0.5)

                // Drive over to pick up another sample
//                .lineToLinearHeading(new Pose2d(5, -37, Math.toRadians(0)))
//                .back(14)
//                .strafeRight(24)

                // Might need tuning

                .addTemporalMarker(() -> {
                    // Grab first additional sample
//                    extendToFirstGrab();
//                    sampleIn(); // blocking!
//                    extendToGround(); // will run after sampleIn() has slept
                })

                // Start moving manipulator to basket position
                .addTemporalMarker(() -> {
//                    pivotToBasket();
//                    extendToBasket();
                })

                // Too far
//                .strafeLeft(24)
//                .forward(13)
//                .turn(Math.toRadians(45))
//                .strafeLeft(2)
//                .forward(3.3)

                // Drive back over to the basket
//                .lineToLinearHeading(new Pose2d(14, -13, Math.toRadians(45)))
//                .strafeLeft(26)
//                .forward(16.3)
//                .turn(Math.toRadians(45))

                .addTemporalMarker(() -> {
                    // Score third sample
//                    scoreSample();
                })

                // Try to get back to the ground before the OpMode ends
                .addTemporalMarker(() -> {
                    waitForPivotToGround();
                    waitForExtendToGround();
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