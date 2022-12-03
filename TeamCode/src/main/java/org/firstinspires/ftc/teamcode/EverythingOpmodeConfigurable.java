//ALoTO 2022-23
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Everything Opmode (Configured normally)", group="Everything Opmode")
//Start+A for driving, Start+B for manipulator
public class EverythingOpmodeConfigurable extends LinearOpMode {

    //Declaring variables for controls
    private double driveAxis;
    private boolean forwardFull;
    private boolean reverseFull;
    private double forwardAxis;
    private double reverseAxis;
    private boolean slowForward;
    private boolean slowReverse;
    private boolean drivePowerUp;
    private boolean drivePowerDown;
    private double turnAxis;
    private boolean turnLeftFull;
    private boolean turnRightFull;
    private double turnLeftAxis;
    private double turnRightAxis;
    private double strafeAxis;
    private boolean strafeLeftFull;
    private boolean strafeRightFull;
    private double strafeLeftAxis;
    private double strafeRightAxis;
    private boolean clawToggle;
    private boolean armToggle;
    private double elevatorAxis;
    private boolean elevatorUpFull;
    private boolean elevatorDownFull;
    private double elevatorUpAxis;
    private double elevatorDownAxis;
    private boolean elevatorHold;
    private boolean elevatorMaxPowerUp;
    private boolean elevatorMaxPowerDown;
    private boolean elevatorTierUp;
    private boolean elevatorTierDown;
    private boolean elevatorEncoderReset;

    //Defining variables for everything else
    private DcMotor elevatorDrive;
    private int elevatorTier;
    private Servo servo1;//Claw Servo
    private Servo servo2;//Arm Servo
    //wheel motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private DigitalChannel upperLimit;
    private DigitalChannel lowerLimit;
    private double elevatorMaxPower = 0.5;//Change this to adjust the max motor power, or use the D-Pad to adjust it at runtime.
    private double maxDrivePower = 0.5;

    private String status;
    private int tier = 0;
    private double dtemp = 0;
    private int temp = 0;
    private double tierMultiplier = 283.33;//How much the tier is multiplied by to get the target position
    double elevatorPower = 0;

    @Override
    public void runOpMode() {
        //Configure controls in this section:
        //Forward/Backward
        driveAxis = 0;//double //UNTESTED
        forwardFull = false;//boolean //UNTESTED
        reverseFull = false;//boolean //UNTESTED
        forwardAxis = gamepad1.right_trigger;//boolean
        reverseAxis = gamepad1.left_trigger;//boolean
        //Slow drive is an all-or-nothing deal, so these must be booleans (on/off).
        slowForward = gamepad1.right_bumper;//boolean
        slowReverse = gamepad1.left_bumper;//boolean
        //New! Like elevator motor power, change drive motor power!
        drivePowerUp = gamepad1.dpad_right;//boolean
        drivePowerDown = gamepad1.dpad_left;//boolean

        //Turning
        turnAxis = gamepad1.left_stick_x;//double
        turnLeftFull = false;//boolean //UNTESTED
        turnRightFull = false;//boolean //UNTESTED
        turnLeftAxis = 0;//double //UNTESTED
        turnRightAxis = 0;//double //UNTESTED

        //Strafing
        strafeAxis = gamepad1.right_stick_x;//double
        strafeLeftFull = false;//boolean //UNTESTED
        strafeRightFull = false;//boolean //UNTESTED
        strafeLeftAxis = 0;//double //UNTESTED
        strafeRightAxis = 0;//double //UNTESTED

        //Manipulator
        clawToggle = gamepad2.a;//boolean
        armToggle = gamepad2.x;//boolean //NOTE: Could become a double in the future, for analog arm
        //position rather than an either-or situation

        //Elevator
        elevatorAxis = 0;//double //UNTESTED
        elevatorUpFull = false;//boolean //UNTESTED
        elevatorDownFull = false;//boolean //UNTESTED
        elevatorUpAxis = gamepad2.right_trigger;//double
        elevatorDownAxis = gamepad2.left_trigger;//double
        elevatorHold = gamepad2.right_bumper;//boolean
        elevatorMaxPowerUp = gamepad2.dpad_up;//boolean
        elevatorMaxPowerDown = gamepad2.dpad_down;//boolena
        //More Elevator - stuff that doesn't directly move it
        elevatorTierUp = false;//boolean //UNTESTED
        elevatorTierDown = false;//boolean //UNTESTED
        elevatorEncoderReset = gamepad2.back;//boolean //UNTESTED


        elevatorDrive = hardwareMap.get(DcMotor.class, "Ellyvader");
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        elevatorDrive.setPower(0);
        elevatorDrive.setTargetPosition(0);
        elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        ElapsedTime elevPowerUpDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime elevPowerDownDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime elevTierUpDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime elevTierDownDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean clawFlag = true;
        boolean armFlag = true;//If arm works funny, try changing this flag.
        servo1.setPosition(0);

        status = "Ready";
        telemetry.update();
        waitForStart();

//Elevator code
        status = "Running";
        while (opModeIsActive()) {
            if (elevatorAxis != 0) {
                elevatorPower = elevatorAxis;
            } else {
                double upPower = Range.clip(elevatorUpAxis, 0, elevatorMaxPower);//Makes sure elevator motor never runs above max. power
                double downPower = (-Range.clip(elevatorDownAxis, 0, elevatorMaxPower)) / 2;//Divide by 2, because elevator is assisted by gravity and fast descent has broken it before!

                //Set elevator motor to power calculated above
//            if (elevatorUpAxis > 0) {
//                elevatorDrive.setPower(upPower);
//            }
                if (elevatorHold) {//Hold elevator up against gravity (largely unused now)
                    elevatorDrive.setPower(.2);
                } else if (elevatorUpFull) {
                    elevatorDrive.setPower(.3);
                } else if (elevatorDownFull) {
                    elevatorDrive.setPower(-.15);
                } else {
                    elevatorPower = upPower - downPower;
                }
//            } else if (elevatorDownAxis > 0) {
//                elevatorDrive.setPower(downPower);
//            }
            }
            elevatorDrive.setPower(elevatorPower);
            //Adjust Elevator Maximum Power
            if (elevatorMaxPowerUp && elevPowerUpDeb.milliseconds() > 500) {//Change max. power that elevator motor will run at
                elevatorMaxPower += 0.1;
                elevPowerUpDeb.reset();
            } else if (elevatorMaxPowerDown && elevPowerDownDeb.milliseconds() < 500) {
                elevatorMaxPower -= 0.1;
                elevPowerDownDeb.reset();

            //Tiers
            } else if (elevatorTierUp && elevatorTier < 2 && elevTierUpDeb.milliseconds() > 1000) {
                elevatorTier += 1;
            } else if (elevatorTierDown && elevatorTier > 0 && elevTierDownDeb.milliseconds() > 1000) {
                elevatorTier -= 1;
                if (tier == 0) {
                    elevatorDrive.setTargetPosition(0);//Code below only runs when tier > 0, so it never turns target back to 0 when tier == 0!
                }

            //Set current encoder position to 0
            } else if (elevatorEncoderReset) {
                elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                sleep(1000);
                elevatorDrive.setTargetPosition(0);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                elevatorDrive.setPower(0);//If elevator is not being commanded, make sure it's stopped.
            }

            //Actually make the motor go to its designated tier (preliminary code...)
            if (tier != 0) {
                dtemp = tier * tierMultiplier;
                int temp = (int)dtemp;
                elevatorDrive.setTargetPosition(temp);
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

            //Slow Driving
            if (slowForward) {
                forwardAxis = 0.25;
            } else if (forwardFull) {
                forwardAxis = 1;
            }
            if (slowReverse) {
                reverseAxis = 0.25;
            } else if (reverseFull) {
                reverseAxis = 1;
            }
            if (turnLeftFull && !turnRightFull) {
                turnAxis = -1;
            } else if (turnRightFull && !turnLeftFull) {
                turnAxis = 1;
            }
            if (strafeLeftFull && !strafeRightFull) {
                strafeAxis = -1;
            } else if (strafeRightFull && !strafeLeftFull) {
                strafeAxis = 1;
            }

            //Change Max Drive Power
            if (drivePowerUp && maxDrivePower < 1 && driveUpDeb.milliseconds() > 500) {
                maxDrivePower += 0.1;
                driveUpDeb.reset();
            } else if (drivePowerDown && maxDrivePower > 0 && driveDownDeb.milliseconds() < 500) {
                maxDrivePower -= 0.1;
                driveDownDeb.reset();
            }

            //Combine controls to allow a mix of all axes - kind of works?
            leftPower    = Range.clip(forwardAxis + driveAxis + strafeAxis - strafeLeftAxis + strafeRightAxis + turnAxis - turnLeftAxis + turnRightAxis - reverseAxis, -maxDrivePower, maxDrivePower) ;//Makes sure motors only run up to half power
            rightPower   = Range.clip(forwardAxis + driveAxis - strafeAxis + strafeLeftAxis - strafeRightAxis - turnAxis + turnLeftAxis - turnRightAxis - reverseAxis, -maxDrivePower, maxDrivePower) ;//Adds all controller inputs together so they can kind of work simultaneously (it doesn't work very well right now)
            leftbackPower = Range.clip(forwardAxis + driveAxis - strafeAxis + strafeLeftAxis - strafeRightAxis + turnAxis - turnLeftAxis + turnRightAxis - reverseAxis, -maxDrivePower, maxDrivePower) ;//I think it's +driveAxis on all of them?
            rightbackPower  = Range.clip(forwardAxis + driveAxis + strafeAxis - strafeLeftAxis + strafeRightAxis - turnAxis + turnLeftAxis - turnRightAxis - reverseAxis , -maxDrivePower, maxDrivePower) ;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftbackPower);
            rightBack.setPower(rightbackPower);

            //Telemetry
            telemetry.addData("Status", status);
            telemetry.addData("Max Drive Power", maxDrivePower);
            telemetry.addData("Max Elevator Power", elevatorMaxPower);
            telemetry.addData("Elevator Tier", elevatorTier);
            telemetry.addData("Elevator Target Position", elevatorDrive.getTargetPosition());
            telemetry.addData("Elevator Actual Position", elevatorDrive.getCurrentPosition());
            telemetry.addData("Elevator RunMode", elevatorDrive.getMode());
            telemetry.update();
        }
    }
}

