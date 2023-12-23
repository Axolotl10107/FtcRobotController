package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadDTS;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadInterface;


@TeleOp(name="Manipulator Opmode (Test Copy)", group="Manipulator Opmode")
public class ManipulatorCodeTestCopy extends LinearOpMode {

    Servo planeServo;
    Servo clawServo;

    //armPivot powers
    double armSlow = 0.15;
    double armMedium = 0.2;
    double gravityDivisor = 2;
    //Example: if armMedium = 0.2 and gravityDivisor = 2, then up power will actually be 0.3
    //and down power will actually be 0.1 - it shifts it up by the power / gravityDivisor
    //Behaves oddly when set to 1 - comment out the lines that use it instead

    private ElapsedTime runtime = new ElapsedTime();
    //    private ArmMotor armPivot = null;
    private DcMotor armPivot;
//    private DcMotor tempMotor = null;
    private DcMotor armExtend = null;
    //motors for driving
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    int elevatorLowerLimit = 0;
    int elevatorUpperLimit = elevatorLowerLimit + 2500;

    int armDefaultPosition = 0;
    double armHoldPower = 0.5;

    double clawServoDefaultPosition = .585;

    double maxDrivePower = 0.7; //change at runtime with driver's start/back buttons
    ElapsedTime maxDrivePowerDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int mDPDmillis = 300;

    Telemetry.Log log = telemetry.log();

    @Override
    public void runOpMode() {
        GamepadDTS controls = new GamepadDTS(gamepad1, gamepad2);

        telemetry.addData("Status", "Ready for Initialization");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

// Plane Servo Code
        planeServo = hardwareMap.get(Servo.class, "planeservo");

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setPosition(clawServoDefaultPosition);

        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);

//        tempMotor = hardwareMap.get(DcMotor.class, "armPivot");
//        tempMotor.setDirection(DcMotor.Direction.REVERSE);
//        armPivot = new ArmMotor(tempMotor);
        armPivot = hardwareMap.get(DcMotor.class, "armPivot");

        //set current position of all motors as 0
        //and as a convenient side-effect disallow them from running during initialization
        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armPivot.setTargetPosition(0); //now that we know its current position is 0
        //If you change armDefaultPosition, _leave this at 0!_ It cannot move during init.
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart(); //after this runs after Start is pressed - *nothing can move before this*!

        runtime.reset();

        //let all the motors run again
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            // Pivot the arm
//            armPivot.runToPosition();

            if (controls.armMovement() != 0) { //slow movement (D-Pad U/D)
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(controls.armMovement() * armSlow);
//                armPivot.setPower((controls.armMovement() * armSlow) + (armSlow / gravityDivisor));
                //swap the commented line to use gravityDivisor, which will offset the power up
            }
            else if (controls.armMediumMovement() != 0) { //medium movement (D-Pad R/L)
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(controls.armMediumMovement() * armMedium);
//                armPivot.setPower((controls.armMediumMovement() * armMedium) + (armMedium / gravityDivisor));
            }
            else if (controls.armFastMovement() != 0) { //fast movement (Left Stick Y-Axis)
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(controls.armFastMovement());
            }
            else if (armPivot.getPower() < armHoldPower - 0.02) { //If we're not moving...
                //Runs once because we change something that will make the condition false
                armPivot.setTargetPosition(armPivot.getCurrentPosition()); //hold current position
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPivot.setPower(0.5); //it may be near 0 from the analog stick returning to center
                //It will go to the target with this much power.
            }
            telemetry.addData("armPivot power", armPivot.getPower());
            //displays info on the Driver Station
            telemetry.addData("armPivot position", armPivot.getCurrentPosition());
            telemetry.addData("armExtend position", armExtend.getCurrentPosition());


            //Change max. drive power
            if (controls.maxDrivePowerUp() == 1 && maxDrivePowerDeb.milliseconds() > mDPDmillis) {
                //If we've waited long enough...
                maxDrivePower += 0.1;
                maxDrivePowerDeb.reset(); //reset stopwatch so we have to wait
            }
            if (controls.maxDrivePowerDown() == 1 && maxDrivePowerDeb.milliseconds() > mDPDmillis) {
                maxDrivePower -= 0.1;
                maxDrivePowerDeb.reset();
            }
            telemetry.addData("Max Drive Power", maxDrivePower);

            if (controls.armForward() > 0) {
                armPivot.setTargetPosition(armDefaultPosition);
            } else if (controls.armBackward() > 0) {
                armPivot.setTargetPosition(armDefaultPosition + 1750);
            }


            // Extend/Retract the arm / elevator, making sure we don't go too far
            if (armExtend.getCurrentPosition() < elevatorLowerLimit) {
                armExtend.setPower(Range.clip(controls.elevatorMovement(), 0, 1));
            } else if (armExtend.getCurrentPosition() > elevatorUpperLimit) {
                armExtend.setPower(Range.clip(controls.elevatorMovement(), -1, 0));
            } else {
                armExtend.setPower(controls.elevatorMovement());

            }


            if (controls.clawOpen() > 0) { //Open the claw
                clawServo.setPosition(clawServoDefaultPosition - .1); //Opens claw
            } else if (controls.clawClose() > 0) { //Close the claw
                clawServo.setPosition(clawServoDefaultPosition); //Closes claw
            }


            // Drive around
            leftFront.setPower((controls.forwardMovement() + controls.strafeMovement() + controls.rotateMovement()) * maxDrivePower);
            rightFront.setPower((controls.forwardMovement() - controls.strafeMovement() - controls.rotateMovement()) * maxDrivePower);
            leftBack.setPower((controls.forwardMovement() - controls.strafeMovement() + controls.rotateMovement()) * maxDrivePower);
            rightBack.setPower((controls.forwardMovement() + controls.strafeMovement() - controls.rotateMovement()) * maxDrivePower);
            //Multiply rather than clip so you can still use the full range of each axis - gives finer control


            //Launch the plane / drone
            if (controls.planeLaunch() == 1) {
                planeServo.setPosition(1); //release the rubberband
            }
            else {
                planeServo.setPosition(0); //return to starting position so we can launch again later
            }

            telemetry.update(); //actually transmit telemetry to the Driver Station
        }
    }
}
