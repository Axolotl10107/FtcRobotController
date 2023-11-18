package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadDTS;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadInputs;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadInterface;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadLinear;


@TeleOp(name="Manipulator Opmode", group="Manipulator Opmode")
public class Manipulator_Code extends LinearOpMode {

    //armPivot speeds
    double armSlow = 0.15;
    double armMedium = 0.2;
    double gravityDivisor = 2;

    private ElapsedTime runtime = new ElapsedTime();
//    private ArmMotor armPivot = null;
    private DcMotor armPivot;
    private DcMotor tempMotor = null;
    private DcMotor armExtend = null;
    //motors for driving
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    double driveClip = 0.7;
    ElapsedTime driveClipDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    Telemetry.Log log = telemetry.log();

    @Override
    public void runOpMode() {
        try {
            realOpMode();
        } catch (Exception x){
            log.add(x.getStackTrace().toString());
            throw x;
        }
    }

    public void realOpMode() {
        GamepadDTS controls = new GamepadDTS(gamepad1, gamepad2);

        telemetry.addData("Status", "Ready for Initialisation");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //code runs the arm up and down
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);

        tempMotor = hardwareMap.get(DcMotor.class, "armPivot");
        tempMotor.setDirection(DcMotor.Direction.REVERSE);
//        armPivot = new ArmMotor(tempMotor);
        armPivot = hardwareMap.get(DcMotor.class, "armPivot");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        int elevatorLowerLimit = armExtend.getCurrentPosition();
        int elevatorUpperLimit = elevatorLowerLimit + 2500;

        int armDefaultPosition = armPivot.getCurrentPosition();
        armPivot.setTargetPosition(armDefaultPosition);
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        int armCurrentPosition = armPivot.getCurrentPosition();

        double servoDefaultPosition = .585;
        clawServo.setPosition(servoDefaultPosition);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("armPivot power", armPivot.getPower());
            telemetry.addData("armPivot position", armPivot.getCurrentPosition());
    // controls the arm
//            armPivot.runToPosition();
//            if (controls.armMovement() != 0) {
//                armCurrentPosition += controls.armMovement();
//                armPivot.setTargetPosition(armCurrentPosition);
//            }

//            if (controls.armMediumMovement() != 0) {
//                armCurrentPosition += controls.armMediumMovement();
//                armPivot.setTargetPosition(armCurrentPosition);
//            }

            if (controls.armMovement() != 0) { //slow movement (D-Pad U/D)
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower((controls.armMovement() * armSlow) + (armSlow / gravityDivisor));
            }
            else if (controls.armMediumMovement() != 0) { //medium movement (D-Pad R/L)
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower((controls.armMediumMovement() * armMedium) + (armMedium / gravityDivisor));
            }
            else if (controls.armFastMovement() != 0) { //fast movement (Left Stick Y-Axis)
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(controls.armFastMovement());
            }
            else { //If we're not moving...
                armPivot.setTargetPosition(armPivot.getCurrentPosition()); //hold current position
//                armCurrentPosition = armPivot.getCurrentPosition();
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPivot.setPower(0.5); //it may be near 0 from the analog stick returning to center
                //It will go to the target with this much power.
            }

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

            if (controls.armForward() > 0) {
                armPivot.setTargetPosition(armDefaultPosition);
            } else if (controls.armBackward() > 0) {
                armPivot.setTargetPosition(armDefaultPosition + 1750);
            }

    // controls the elevator
            if (armExtend.getCurrentPosition() < elevatorLowerLimit) {
                armExtend.setPower(Range.clip(controls.elevatorMovement(), 0,1));
            } else if (armExtend.getCurrentPosition() > elevatorUpperLimit) {
                armExtend.setPower(Range.clip(controls.elevatorMovement(), -1, 0));
            } else {
                armExtend.setPower(controls.elevatorMovement());

            }

    // controls the claw
            if (controls.clawOpen() > 0) {
                clawServo.setPosition(servoDefaultPosition - .1); //Opens claw
            } else if (controls.clawClose() > 0) {
                clawServo.setPosition(servoDefaultPosition); //Closes claw
            }
    // controls the wheels
            leftFront.setPower(Range.clip(controls.forwardMovement() + controls.strafeMovement() + controls.rotateMovement(), -driveClip, driveClip));
            rightFront.setPower(Range.clip(controls.forwardMovement() - controls.strafeMovement() - controls.rotateMovement(), -driveClip, driveClip));
            leftBack.setPower(Range.clip(controls.forwardMovement() - controls.strafeMovement() + controls.rotateMovement(), -driveClip, driveClip));
            rightBack.setPower(Range.clip(controls.forwardMovement() + controls.strafeMovement() - controls.rotateMovement(), -driveClip, driveClip));

            telemetry.update();
        }
    }
}
