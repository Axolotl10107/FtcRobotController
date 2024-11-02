package org.firstinspires.ftc.teamcode.fy24.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fy24.controls.GamepadDTS;


@TeleOp(name="TestOpMode")
public class TestOpMode extends LinearOpMode {

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
        armLeftPivot.setDirection(DcMotor.Direction.FORWARD);
        armRightPivot.setDirection(DcMotorSimple.Direction.REVERSE);
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

        CRServo servoClaw;
        servoClaw = hardwareMap.get(CRServo.class, "clawServo");

        double armExtendSpeed = 1110;
        double armPivotSpeed = 1115;

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

        while (opModeIsActive()) {

            if (gamepad1.start && driveClipDeb.milliseconds() > 300) {
                driveClip += 0.1;
                driveClipDeb.reset();
            }
            if (gamepad1.back && driveClipDeb.milliseconds() > 300) {
                driveClip -= 0.1;
                driveClipDeb.reset();
            }
            telemetry.addData("Max Drive Power", driveClip);

            // Emergency Brake

            if (controls.emergencyBrakeX() != 0 && controls.emergencyBrakeA() != 0) {
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

            if (controls.armForward() != 0) {
                armLeftExtend.setVelocity(armExtendSpeed);
                armRightExtend.setVelocity(armExtendSpeed);
            } else if (controls.armBackward() != 0) {
                armLeftExtend.setVelocity(-armExtendSpeed);
                armRightExtend.setVelocity(-armExtendSpeed);
            } else {
                armLeftExtend.setVelocity(0);
                armRightExtend.setVelocity(0);
            }

            if (controls.armPivot() < 0) {
                armLeftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
                armRightPivot.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                armLeftPivot.setDirection(DcMotorSimple.Direction.FORWARD);
                armRightPivot.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if (controls.armPivot() != 0) {
                armLeftPivot.setVelocity(armPivotSpeed * controls.armPivot());
                armRightPivot.setVelocity(armPivotSpeed * controls.armPivot());
            } else {
                armRightPivot.setVelocity(0);
                armLeftPivot.setVelocity(0);
            }

            // Control active intake

            if (controls.intakeServoIn() != 0) {
                servoClaw.setPower(1);
            } else if (controls.intakeServoOut() != 0) {
                servoClaw.setPower(-1);
            } else {
                servoClaw.setPower(0);
            }

            telemetry.addData("Power: ", servoClaw.getPower());
            telemetry.addData("Pivot Input: ", controls.armPivot());
            telemetry.addData("Expected Pivot Velocity: ", armPivotSpeed * controls.armPivot());
            telemetry.addData("Actual Pivot Velocity: ", armRightPivot.getVelocity());
            telemetry.update();
        }
    }
}
