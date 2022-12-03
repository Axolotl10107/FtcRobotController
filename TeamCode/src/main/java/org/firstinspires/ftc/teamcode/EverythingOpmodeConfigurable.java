//ALoTO 2022-23
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.security.spec.ECParameterSpec;

@TeleOp(name="Everything Opmode (Configured normally)", group="Linear Opmode")
//Start+A for driving, Start+B for manipulator
public class EverythingOpmodeConfigurable extends LinearOpMode {

    //Declaring variables for controls
    private double driveAxis;
    private double forwardAxis;
    private double reverseAxis;
    private boolean slowForward;
    private boolean slowReverse;
    private boolean drivePowerUp;
    private boolean drivePowerDown;
    private double turnAxis;
    private double turnLeft;
    private double turnRight;
    private double strafeAxis;
    private double strafeLeft;
    private double strafeRight;
    private boolean clawToggle;
    private boolean armToggle;
    private double elevatorAxis;
    private double elevatorUpAxis;
    private double elevatorDownAxis;
    private boolean elevatorHold;
    private boolean elevatorMaxPowerUp;
    private boolean elevatorMaxPowerDown;
    private boolean elevatorEncoderReset;

    //Defining variables for everything else
    private DcMotor elevatorDrive;
    private Servo servo1;//Claw Servo
    private Servo servo2;//Arm Servo
    //wheel motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private DigitalChannel upperLimit;
    private DigitalChannel lowerLimit;
    private double maxPower = 0.5;//Change this to adjust the max motor power, or use the D-Pad to adjust it at runtime.
    private double maxDrivePower = 0.5;

    @Override
    public void runOpMode() {
        //Configure controls in this section:
        //Forward/Backward
        driveAxis = 0;
        forwardAxis = gamepad1.right_trigger;
        reverseAxis = gamepad1.left_trigger;
        //Slow drive is an all-or-nothing deal, so these must be booleans (on/off).
        slowForward = gamepad1.right_bumper;
        slowReverse = gamepad1.left_bumper;
        //New! Like elevator motor power, change drive motor power!
        drivePowerUp = gamepad1.dpad_right;
        drivePowerDown = gamepad1.dpad_left;

        //Turning
        turnAxis = gamepad1.left_stick_x;
        turnLeft = 0;
        turnRight = 0;

        //Strafing
        strafeAxis = gamepad1.right_stick_x;
        strafeLeft = 0;
        strafeRight = 0;

        //Manipulator
        // |||These are booleans for now. They could possibly become doubles later!|||
        clawToggle = gamepad2.a;
        armToggle = gamepad2.x;

        //Elevator
        elevatorAxis = 0;
        elevatorUpAxis = gamepad2.right_trigger;
        elevatorDownAxis = gamepad2.left_trigger;
        elevatorHold = gamepad2.right_bumper;
        elevatorMaxPowerUp = gamepad2.dpad_up;
        elevatorMaxPowerDown = gamepad2.dpad_down;
        //Untested!
        elevatorEncoderReset = gamepad2.back;


        elevatorDrive = hardwareMap.get(DcMotor.class, "Ellyvader");
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        upperLimit = hardwareMap.get(DigitalChannel.class, "UpperLimit");
        upperLimit.setMode(DigitalChannel.Mode.INPUT);
        lowerLimit = hardwareMap.get(DigitalChannel.class, "LowerLimit");
        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.FORWARD);
//        servo1.setPosition(0);
//        servo2.setPosition(0);
        //wheel motor names
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE );
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        ServoController scont = servo1.getController();
        scont.pwmEnable();
        ElapsedTime clawDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime driveUpDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime driveDownDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime armDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime elevUpDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime elevDownDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean clawFlag = true;
        boolean armFlag = true;//If arm works funny, try changing this flag.
        servo1.setPosition(0);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

//Elevator code
        while (opModeIsActive()) {
            double upPower = Range.clip(elevatorUpAxis, 0, maxPower);//Makes sure elevator motor never runs above max. power
            double downPower = (-Range.clip(elevatorDownAxis, 0, maxPower))/2;
            telemetry.addData("Max Elevator Power:", maxPower);
            if (elevatorUpAxis > 0) {
                elevatorDrive.setPower(upPower);
            }
            else if (elevatorHold) {//Hold elevator up against gravity
                elevatorDrive.setPower(.2);
            }
            else if (elevatorDownAxis > 0) {
                elevatorDrive.setPower(downPower);
            } else if (elevatorMaxPowerUp && elevUpDeb.milliseconds() > 500) {//Change max. power that elevator motor will run at
                maxPower += 0.1;
                elevUpDeb.reset();
            } else if (elevatorMaxPowerDown && elevDownDeb.milliseconds() < 500) {
                maxPower -= 0.1;
                elevDownDeb.reset();
//            } else if (elevatorEncoderReset) { //Not so fast, myself! You haven't actually put that code in here yet?
//                elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                sleep(1000);
//                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                elevatorDrive.setPower(0);//If elevator is not being commanded, make sure it's stopped.
            }

            //Claw Code
            if (clawToggle && clawDeb.milliseconds() > 500) {
                if (clawFlag) {
                    servo1.setPosition(.26);//Closes claw
                } else {
                    servo1.setPosition(0);
                }
                clawFlag = !clawFlag;
                clawDeb.reset();
            }
            telemetry.addData("clawFlag", clawFlag);

            //Arm Code
            if (armToggle && armDeb.milliseconds() > 500) {
                if (armFlag) {
                    servo2.setPosition(0);//Sends arm all the way [].
                } else {
                    servo2.setPosition(.6);
                }
                armDeb.reset();
                armFlag = !armFlag;
            }

            //driving code
            double leftPower;
            double rightPower;
            double leftbackPower;
            double rightbackPower;

            double drive = forwardAxis;
            double negative = reverseAxis;
            double strafe  =  strafeAxis;
            double turn = turnAxis;

            //Half-speed Driving
            if (slowForward) {
                drive = 0.25;
            }
            if (slowReverse) {
                negative = 0.25;
            }

            //Change Max Drive Power
            if (maxDrivePower < 1 && drivePowerUp && driveUpDeb.milliseconds() > 500) {
                maxDrivePower += 0.1;
                driveUpDeb.reset();
            } else if (maxDrivePower > 0 && drivePowerDown && driveDownDeb.milliseconds() < 500) {
                maxDrivePower -= 0.1;
                driveDownDeb.reset();
            }

//            leftPower    = Range.clip(drive + strafe + turn - negative, -.5, .5) ;//Makes sure motors only run up to half power
//            rightPower   = Range.clip(drive - strafe - turn - negative, -.5, .5) ;//Adds all controller inputs together so they can kind of work simultaneously (it doesn't work very well right now)
//            leftbackPower = Range.clip(drive - strafe + turn - negative, -.5, .5) ;
//            rightbackPower  = Range.clip(drive + strafe - turn - negative , -.5, .5) ;

            leftPower    = Range.clip(drive + strafe + turn - negative, -maxDrivePower, maxDrivePower) ;//Makes sure motors only run up to half power
            rightPower   = Range.clip(drive - strafe - turn - negative, -maxDrivePower, maxDrivePower) ;//Adds all controller inputs together so they can kind of work simultaneously (it doesn't work very well right now)
            leftbackPower = Range.clip(drive - strafe + turn - negative, -maxDrivePower, maxDrivePower) ;
            rightbackPower  = Range.clip(drive + strafe - turn - negative , -maxDrivePower, maxDrivePower) ;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftbackPower);
            rightBack.setPower(rightbackPower);

            telemetry.update();
        }
    }
}

