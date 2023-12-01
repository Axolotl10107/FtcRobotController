package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadDTS;


@TeleOp(name="Manipulator Opmode", group="Manipulator Opmode")
public class Manipulator_Code extends LinearOpMode {

    Servo servo123456789;
    //armPivot speeds
    double armSlow = 0.15;
    double armMedium = 0.2;
    double gravityDivisor = 1;

    private ElapsedTime runtime = new ElapsedTime();
    //    private ArmMotor armPivot = null;
    private DcMotor armPivot;
    private Servo clawServo;
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
        } catch (Exception x) {
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
// Plane Servo Code
        servo123456789 = hardwareMap.get(Servo.class, "planeservo");

        //code runs the arm up and down
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);

        tempMotor = hardwareMap.get(DcMotor.class, "armPivot");
        tempMotor.setDirection(DcMotor.Direction.REVERSE);
//        armPivot = new ArmMotor(tempMotor);
        armPivot = hardwareMap.get(DcMotor.class, "armPivot");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        int elevatorLowerLimit = 0;
        int elevatorUpperLimit = elevatorLowerLimit + 2500;

        TouchSensor armCalibration = hardwareMap.get(TouchSensor.class, "armCalibration");
        boolean armCalibrated = true; // should be false, currently set to true to disable
        int armDefaultPosition = 0;
        armPivot.setTargetPosition(armDefaultPosition);
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        int armCurrentPosition = armPivot.getCurrentPosition();

        double servoDefaultPosition = .225;
        clawServo.setPosition(servoDefaultPosition);

        //set current position of all motors as 0
        //and as a convenient side-effect disallow them from running during initialization
        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        runtime.reset();

        //let all the motors run again
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            telemetry.addData("armPivot power", armPivot.getPower());
            telemetry.addData("armPivot position", armPivot.getCurrentPosition());
            telemetry.addData("armExtend position", armExtend.getCurrentPosition());
            telemetry.addData("arm calibrated", armCalibrated);
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



            if (controls.armMovement() != 0 && armCalibrated) { //slow movement (D-Pad U/D)
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(controls.armMovement() * armSlow);
//                armPivot.setPower((controls.armMovement() * armSlow) + (armSlow / gravityDivisor));
            } else if (controls.armMediumMovement() != 0 && armCalibrated) { //medium movement (D-Pad R/L)
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(controls.armMediumMovement() * armMedium);
//                armPivot.setPower((controls.armMediumMovement() * armMedium) + (armMedium / gravityDivisor));
            } else if (controls.armFastMovement() != 0 && armCalibrated) { //fast movement (Left Stick Y-Axis)
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(controls.armFastMovement());
            } else if (armPivot.getPower() < 0.95 && armCalibrated) { //If we're not moving...
                //This should run once because we'll set the power to 0.5 here.
                armPivot.setTargetPosition(armPivot.getCurrentPosition()); //hold current position
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPivot.setPower(1); //it may be near 0 from the analog stick returning to center
                //It will go to the target with this much power.
            }

            if (armCalibrated == false) {
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(-0.5);
                if (armCalibration.isPressed() || armCalibration.getValue() != 0 || (runtime.milliseconds() > 5000)) {
                    armPivot.setPower(0);
                    armCalibrated = true;
                    armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
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
                armExtend.setPower(Range.clip(controls.elevatorMovement(), 0, 1));
            } else if (armExtend.getCurrentPosition() > elevatorUpperLimit) {
                armExtend.setPower(Range.clip(controls.elevatorMovement(), -1, 0));
            } else {
                armExtend.setPower(controls.elevatorMovement());

            }

            // controls the claw
            if (controls.clawOpen() != 0) {
                clawServo.setPosition(servoDefaultPosition - .1); //Opens claw
            } else if (controls.clawClose() != 0) {
                clawServo.setPosition(servoDefaultPosition); //Closes claw
            }
            // controls the wheels
            leftFront.setPower((controls.forwardMovement() + controls.strafeMovement() + controls.rotateMovement()) * driveClip);
            rightFront.setPower((controls.forwardMovement() - controls.strafeMovement() - controls.rotateMovement()) * driveClip);
            leftBack.setPower((controls.forwardMovement() - controls.strafeMovement() + controls.rotateMovement()) * driveClip);
            rightBack.setPower((controls.forwardMovement() + controls.strafeMovement() - controls.rotateMovement()) * driveClip);

            telemetry.update();

           // Plane Servo Code Continue

            if (gamepad1.right_bumper) {
                servo123456789.setPosition(1);
            }
            else {
                servo123456789.setPosition(0);
            }
        }
    }
}
