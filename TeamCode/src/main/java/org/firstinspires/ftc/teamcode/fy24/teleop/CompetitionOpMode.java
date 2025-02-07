package org.firstinspires.ftc.teamcode.fy24.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.fy24.controls.GamepadDTS;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name="In Theory 24-25 Competition TeleOp", group="TeleOp24")
public class CompetitionOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //    private ArmMotor armPivot = null;
    //motors for driving
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    // Arm motors
    private DcMotorEx armLeftExtend = null;
    private DcMotorEx armRightExtend = null;
    private DcMotorEx armLeftPivot = null;
    private DcMotorEx armRightPivot = null;

    double driveClip = 0.7;
    ElapsedTime driveClipDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    Telemetry.Log log = telemetry.log();

    @Override
    public void runOpMode() {
        try {
            realOpMode();
        } catch (Exception x) {
            log.add(x.getStackTrace().toString());
            throw x;
        }
    }

    final double limitBuffer = 2;
    final double motorLength = 3.50;
    final double horizontalLimit = 42 - motorLength - limitBuffer;
    final double ticksPerInch = 157.86;
    final double ticksPerDegree = 32.06;

    double pivotPos;
    double armPos;
    public boolean checkArmLimit(Double angle) {
        // horizontalLimit / Math.cos(Math.toRadians(angle))
        if (armPos < Math.abs((1 / Math.cos(Math.toRadians(angle))) * horizontalLimit)) {
            return true;
        } else {
            return false;
        }
    }

    public void pivotToNormalMode() {
        armLeftPivot.setPower(0);
        armRightPivot.setPower(0);
        armLeftPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRightPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extendToNormalMode() {
        armLeftExtend.setPower(0);
        armRightExtend.setPower(0);
        armLeftExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRightExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void realOpMode() {
        GamepadDTS controls = new GamepadDTS(gamepad1, gamepad2);

        telemetry.addData("Status", "Ready for Initialisation");

        armLeftExtend = hardwareMap.get(DcMotorEx.class, "armLeftExtend");
        armRightExtend = hardwareMap.get(DcMotorEx.class, "armRightExtend");
        armLeftPivot = hardwareMap.get(DcMotorEx.class, "armLeftPivot");
        armRightPivot = hardwareMap.get(DcMotorEx.class, "armRightPivot");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        armLeftExtend.setDirection(DcMotor.Direction.FORWARD);
        armRightExtend.setDirection(DcMotor.Direction.REVERSE);
        armLeftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armRightPivot.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        armLeftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        CRServo servoIntake;
        Servo servoClawPivot;
        Servo servoClaw;
        servoIntake = hardwareMap.get(CRServo.class, "intakeServo");
        servoClawPivot = hardwareMap.get(Servo.class, "clawPivotServo");
        servoClaw = hardwareMap.get(Servo.class, "clawServo");

        double armExtendSpeed = 1600;
        double armPivotSpeed = 1600;
        double driveSpeed = 1;

        waitForStart();

        runtime.reset();

        armLeftExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRightExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRightExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            armPos = (((armLeftExtend.getCurrentPosition() + armRightExtend.getCurrentPosition()) / 2) / ticksPerInch) + 17.5;
            pivotPos = Math.abs(((armLeftPivot.getCurrentPosition() + armRightPivot.getCurrentPosition()) / 2) / ticksPerDegree);

            if (gamepad1.start && driveClip < 1 && driveClipDeb.milliseconds() > 300) {
                driveClip += 0.1;
                driveClipDeb.reset();
            }
            if (gamepad1.back && driveClip > 0.25 && driveClipDeb.milliseconds() > 300) {
                driveClip -= 0.1;
                driveClipDeb.reset();
            }
            telemetry.addData("Max Drive Power", driveClip);

            // Change speed

            if (controls.driveSpeedUp() != 0 && driveClip < 1 && driveClipDeb.milliseconds() > 300) {
                driveClip = 1;
                driveClipDeb.reset();
            } else if (controls.driveSpeedDown() != 0 && driveClip > 0.25 && driveClipDeb.milliseconds() > 300) {
                driveClip = 0.3;
                driveClipDeb.reset();
            }

            // Reset arm vars

            if (controls.resetArmVars() != 0) {
                armRightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLeftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armRightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLeftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Brake

            if (controls.brake() != 0) {
                double currentVelocity = leftFront.getVelocity();
                while (leftFront.getVelocity() > 0 && currentVelocity > 0) {
                    leftFront.setVelocity(-leftFront.getVelocity());
                    rightFront.setVelocity(-rightFront.getVelocity());
                    leftBack.setVelocity(-leftBack.getVelocity());
                    rightBack.setVelocity(-rightBack.getVelocity());
                }
                while (leftFront.getVelocity() < 0 && currentVelocity < 0) {
                    leftFront.setVelocity(-leftFront.getVelocity());
                    rightFront.setVelocity(-rightFront.getVelocity());
                    leftBack.setVelocity(-leftBack.getVelocity());
                    rightBack.setVelocity(-rightBack.getVelocity());
                }
                leftFront.setVelocity(0);
                rightFront.setVelocity(0);
                leftBack.setVelocity(0);
                rightBack.setVelocity(0);
                leftFront.setMotorDisable();
                rightFront.setMotorDisable();
                leftBack.setMotorDisable();
                rightBack.setMotorDisable();
            } else {

                // controls the wheels
                leftFront.setMotorEnable();
                rightFront.setMotorEnable();
                leftBack.setMotorEnable();
                rightBack.setMotorEnable();
                leftFront.setPower((controls.forwardMovement() + controls.strafeMovement() + controls.rotateMovement()) * driveClip);
                rightFront.setPower((controls.forwardMovement() - controls.strafeMovement() - controls.rotateMovement()) * driveClip);
                leftBack.setPower((controls.forwardMovement() - controls.strafeMovement() + controls.rotateMovement()) * driveClip);
                rightBack.setPower((controls.forwardMovement() + controls.strafeMovement() - controls.rotateMovement()) * driveClip);

            }
            // Control arm
            if (!checkArmLimit(pivotPos) && Math.abs(pivotPos) <= 75) {
                // TODO: bug - pivot takes off around position abs(2000) or something if we don't set its velocity to 0
                // TODO: Put controller movement above limit check so it doesn't bounce when you try to push it out too far
                while (!checkArmLimit(pivotPos)) {
                    armPos = ( (double) ((armLeftExtend.getCurrentPosition() + armRightExtend.getCurrentPosition()) / 2) / ticksPerInch) + 17.5;
                    pivotPos = Math.abs(( (double) (armLeftPivot.getCurrentPosition() + armRightPivot.getCurrentPosition()) / 2) / ticksPerDegree);
                    armLeftExtend.setVelocity(-armExtendSpeed);
                    armRightExtend.setVelocity(-armExtendSpeed);
                    armLeftPivot.setVelocity(0);
                    armRightPivot.setVelocity(0);
                }
//                armLeftExtend.setVelocity(0);
//                armRightExtend.setVelocity(0);
            } else if (controls.armForward() != 0 && checkArmLimit(pivotPos)) {
                extendToNormalMode();
                armLeftExtend.setVelocity(armExtendSpeed);
                armRightExtend.setVelocity(armExtendSpeed);
            } else if (controls.armBackward() != 0) {
                extendToNormalMode();
                armLeftExtend.setVelocity(-armExtendSpeed);
                armRightExtend.setVelocity(-armExtendSpeed);
            } else {
//                armLeftExtend.setVelocity(0);
//                armRightExtend.setVelocity(0);
                armLeftExtend.setTargetPosition(armLeftExtend.getCurrentPosition());
                armRightExtend.setTargetPosition(armRightExtend.getCurrentPosition());
                armLeftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLeftExtend.setPower(1);
                armRightExtend.setPower(1);
            }

            if (controls.armPivot() > 0 && (pivotPos < 90)) {
                pivotToNormalMode();
                armLeftPivot.setVelocity(armPivotSpeed);
                armRightPivot.setVelocity(armPivotSpeed);
            } else if (controls.armPivot() < 0) {
                pivotToNormalMode();
                armLeftPivot.setVelocity(-armPivotSpeed);
                armRightPivot.setVelocity(-armPivotSpeed);
            } else {
//                armRightPivot.setPower(0);
//                armLeftPivot.setPower(0);
                armRightPivot.setTargetPosition(armRightPivot.getCurrentPosition());
                armLeftPivot.setTargetPosition(armLeftPivot.getCurrentPosition());
                armRightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLeftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRightPivot.setPower(1);
                armLeftPivot.setPower(1);
            }

            // Control active intake

            if (controls.intakeServoIn() != 0) {
                servoIntake.setPower(1);
            } else if (controls.intakeServoOut() != 0) {
                servoIntake.setPower(-1);
            } else {
                servoIntake.setPower(0);
            }

            if (controls.clawServoIn() != 0) {
                telemetry.addLine("in");
                servoClaw.setPosition(0);
            } else if (controls.clawServoOut() != 0) {
                telemetry.addLine("out");
                servoClaw.setPosition(0.5);
            }

            if (controls.clawPivotUp() != 0) {
                servoClawPivot.setPosition(servoClawPivot.getPosition() + 0.1);
            } else if (controls.clawPivotDown() != 0) {
                servoClawPivot.setPosition(servoClawPivot.getPosition() - 0.1);
            }

            telemetry.addData("Position", servoClawPivot.getPosition());
            telemetry.addData("Pivot Input", controls.armPivot());
            telemetry.addData("Expected Pivot Velocity", armPivotSpeed * controls.armPivot());
            telemetry.addData("Actual Pivot Velocity", armRightPivot.getVelocity());
            telemetry.addData("Left Extend", armLeftExtend.getCurrentPosition());
            telemetry.addData("Right Extend", armRightExtend.getCurrentPosition());
            telemetry.addData("Arm Limit", Math.abs((1 / Math.cos(Math.toRadians(pivotPos))) * horizontalLimit));
            telemetry.addData("Extension Input", controls.armForward() - controls.armBackward());
            telemetry.addData("Arm Pos", armPos);
            telemetry.addData("Pivot Pos Left", armLeftPivot.getCurrentPosition());
            telemetry.addData("Pos Pos Right", armRightPivot.getCurrentPosition());
            telemetry.addData("Pivot Pos", (armLeftPivot.getCurrentPosition() + armRightPivot.getCurrentPosition()) / 2);
            telemetry.addData("Pivot Pos Degrees", pivotPos);
            telemetry.addData("Pivot Input", controls.armPivot());
            telemetry.addData("Claw Servo", controls.clawServoIn() - controls.clawServoOut());
            telemetry.update();
        }
    }
}
