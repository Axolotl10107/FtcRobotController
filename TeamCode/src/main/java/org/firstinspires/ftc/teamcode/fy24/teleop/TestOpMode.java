package org.firstinspires.ftc.teamcode.fy24.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    // Arm motors
    private DcMotorEx armLeftExtend = null;
    private DcMotorEx armRightExtend = null;
//    private DcMotorEx armLeftPivot = null;
//    private DcMotorEx armRightPivot = null;

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
//        armLeftPivot = hardwareMap.get(DcMotor.class, "armLeftPivot");
//        armRightPivot = hardwareMap.get(DcMotor.class, "armRightPivot");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        armLeftExtend.setDirection(DcMotor.Direction.FORWARD);
        armRightExtend.setDirection(DcMotor.Direction.REVERSE);
//        armLeftPivot.setDirection(DcMotor.Direction.FORWARD);
//        armRightPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        armLeftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armLeftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armRightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double armExtendSpeed = 1110;
        double armPivotSpeed = 0.15;

        waitForStart();

        runtime.reset();

        armLeftExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRightExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armLeftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armRightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            /*
            Consider this - might not be what you want but it's a potential option:
            if (controls.armMovement() != 0) {
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(controls.armMovement());
            } else {
                armPivot.setTargetPosition(armPivot.getCurrentPosition());
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            */
            // controls the wheels
            leftFront.setPower((controls.forwardMovement() + controls.strafeMovement() + controls.rotateMovement()) * driveClip);
//            rightFront.setPower((controls.forwardMovement() - controls.strafeMovement() - controls.rotateMovement()) * driveClip);
            leftBack.setPower((controls.forwardMovement() - controls.strafeMovement() + controls.rotateMovement()) * driveClip);
//            rightBack.setPower((controls.forwardMovement() + controls.strafeMovement() - controls.rotateMovement()) * driveClip);

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

//            armLeftPivot.setPower(armPivotSpeed * controls.armPivot());
//            armRightPivot.setPower(armPivotSpeed * controls.armPivot());

            telemetry.update();
        }
    }
}
