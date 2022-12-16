//ALoTO 2022-23
package org.firstinspires.ftc.teamcode;

//TODO: Elevator very broken (going up just jitters, tiers do nothing)
//TODO: Elevator max is 1000 now with new rigging!
//TODO: Tier power is getting set for about .3 seconds then going back to 0, even when no line to set back to 0 is active in code? Check JavaDoc on relevant functions.

//Bugs:
//RESOLVED - Motors keep running when sticks return to 0
//RESOLVED (accidentally used less than instead of greater than - < instead of > - for debounce timer) drivePowerDown not working
//Elevator up/down analog working weird - TODO: needs to switch runmode!

//Additional Notes:
/*
* - UpperLimit and LowerLimit are gone, since they have not been implemented in hardware.
*   - TODO: Replace UpperLimit and LowerLimit with encoders (limit value vars now exist!)
* */

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

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    //Declaring variables for controls
    //Drive (forward/backward)
    private double driveAxis;
    private boolean forwardFull;
    private static final double forwardFullPower = 1.0;
    private boolean reverseFull;
    private static final double reverseFullPower = -1.0;
    private double forwardAxis;
    private double reverseAxis;
    private boolean forwardSlow;
    private static final double forwardSlowPower = 0.25;
    private boolean reverseSlow;
    private static final double reverseSlowPower = -0.25;
    private boolean drivePowerUp;
    private boolean drivePowerDown;

    //Turn
    private double turnAxis;
    private boolean turnLeftFull;
    private static final double turnLeftFullPower = -1.0;
    private boolean turnRightFull;
    private static final double turnRightFullPower = 1.0;
    private double turnLeftAxis;
    private double turnRightAxis;

    //Strafe
    private double strafeAxis;
    private boolean strafeLeftFull;
    private static final double strafeLeftFullPower = -1.0;
    private boolean strafeRightFull;
    private static final double strafeRightFullPower = 1.0;
    private double strafeLeftAxis;
    private double strafeRightAxis;

    //Manipulator - TODO: Needs radical expansion
    private boolean clawToggle;
    private boolean armToggle;
    //Values
    private static final double clawClosedPosition = 0.26;
    private static final double clawOpenPosition = 0;
    private static final double armLeftPosition = 0.1;// TODO: Left/right are definitely swapped (New: Left=Right - they are wrong)
    private static final double armRightPosition = 0.6;
    private static final int manipulatorToggleTimer = 500;

    //Elevator
    private double elevatorAxis;
    private static final double elevatorAxisUpperLimit = 1.0;
    private static final double elevatorAxisLowerLimit = 0.5;//Elevator down is assisted by gravity; use lower power.
    private boolean elevatorUpFull;
    private static final double elevatorUpFullPower = 0.3;
    private boolean elevatorDownFull;
    private static final double elevatorDownFullPower = -0.15;
    private double elevatorUpAxis;
    private double elevatorDownAxis;
    private boolean elevatorHold;
    private boolean elevatorMaxPowerUp;
    private boolean elevatorMaxPowerDown;
    private static final int maxPowerToggleTimer = 500;//in milliseconds, both drive and elevator
    private boolean elevatorTierUp;
    private boolean elevatorTierDown;
    private static final int elevatorTierToggleTimer = 1000;
    private boolean elevatorEncoderReset;
    private static final double elevatorHoldPower = 0.2;
    //Upper/Lower Limits - tiers are currently derived from these (tier 0 is lower, tier 3 is upper, tiers 1 and 2 are right in between)
    private static final double elevatorPositionUpperLimit = 850;
    private static final double elevatorPositionLowerLimit = 150;

    //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //Declaring variables or creating objects for everything else
    private DcMotor elevatorDrive;
    private int elevatorTier;
    private Servo clawServo;
    private Servo armServo;
    //wheel motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private double elevatorMaxPower = 0.5;//Change this to adjust the max motor power, or use the D-Pad to adjust it at runtime.
    private double maxDrivePower = 0.5;

    //Declaring or initializing various variables used later
    private String status;
//    private int tier = 0;
    private double dtemp = 0;
    private int temp = 0;
    private double tierMultiplier = 283.33;//How much the tier is multiplied by to get the target position
    private double elevatorPower = 0;
    private double strafePower;
    private double turnPower;
    private double upPower;
    private double downPower;

    private void setControls() {
        //   ||| CURRENTLY configured to this layout: https://wordenhome.neocities.org/rd/lm/cl125.html |||
        //Configure controls in this section:
        //Forward/Backward
        /*TODO: Put things that override other things under anything they depend on (what they
        override and what they use to decide how to override); this avoids extra checks later.*/
        driveAxis = 0;//double //UNTESTED
        forwardFull = false;//boolean //UNTESTED
        reverseFull = false;//boolean //UNTESTED
        forwardAxis = gamepad1.right_trigger;//boolean
        reverseAxis = gamepad1.left_trigger;//boolean
        //Slow drive is an all-or-nothing deal, so these must be booleans (on/off).
        forwardSlow = gamepad1.right_bumper;//boolean
        reverseSlow = gamepad1.left_bumper;//boolean

        //Drive Miscellany
        drivePowerUp = gamepad1.dpad_up;//boolean
        drivePowerDown = gamepad1.dpad_down;//boolean

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
        armToggle = gamepad2.x;//boolean //TODO: Absolutely MUST become a double in the future, for
        //analog arm position rather than an either-or situation

        //Elevator
        elevatorAxis = 0;//double //UNTESTED
        elevatorUpFull = false;//boolean //UNTESTED
        elevatorDownFull = false;//boolean //UNTESTED
        elevatorUpAxis = gamepad2.right_trigger;//double
        elevatorDownAxis = gamepad2.left_trigger;//double
        elevatorHold = gamepad2.b;//boolean //NOTE: Rarely Used, may be removed soon
        elevatorMaxPowerUp = gamepad2.dpad_up;//boolean
        elevatorMaxPowerDown = gamepad2.dpad_down;//boolean

        //Elevator Miscellany
        elevatorTierUp = gamepad2.right_bumper;//boolean //BROKEN - 12-15-22
        elevatorTierDown = gamepad2.left_bumper;//boolean //BROKEN - 12-15-22
        elevatorEncoderReset = gamepad2.back;//boolean
    }

    @Override
    public void runOpMode() {
        //Hardware Map
        elevatorDrive = hardwareMap.get(DcMotor.class, "Ellyvader");
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        elevatorDrive.setPower(0);
        elevatorDrive.setTargetPosition(0);
//        elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawServo = hardwareMap.get(Servo.class, "servo1");
        armServo = hardwareMap.get(Servo.class, "servo2");
        clawServo.setDirection(Servo.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
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

        //Make sure the servo PWM is actually enabled - it's not guaranteed.
        ServoController scont = clawServo.getController();
        scont.pwmEnable();

        //Create all the toggle timers
        ElapsedTime clawDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime driveUpDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime driveDownDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime armDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime elevPowerUpDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime elevPowerDownDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime elevTierUpDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime elevTierDownDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        //Flags to track claw/arm positions (TODO: These need to die in favor of analog position)
        boolean clawFlag = true;
        boolean armFlag = true;//If arm works funny, try changing this flag.
//        clawServo.setPosition(0);

        status = "Ready";
        telemetry.update();
        waitForStart();

        //Once play button is pressed...
        status = "Running";
        while (opModeIsActive()) {

            setControls();//Get the values of all the control inputs.

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            //Elevator Code
            if (elevatorAxis != 0) {//If manipulator is using the elevator axis...
                elevatorPower = Range.clip(elevatorAxis, elevatorAxisUpperLimit, elevatorAxisLowerLimit);//use it directly.
            } else {
                upPower = Range.clip(elevatorUpAxis, 0, elevatorMaxPower);//Makes sure elevator motor never runs above max. power
                downPower = Range.clip(elevatorDownAxis, 0, elevatorMaxPower) / 2;//Divide by 2, because elevator is assisted by gravity and fast descent has broken it before!
                if (elevatorUpFull) {
                    elevatorDrive.setPower(elevatorUpFullPower);
                } else if (elevatorDownFull) {
                    elevatorDrive.setPower(elevatorDownFullPower);
                } else if (upPower - downPower == 0) {
                    elevatorDrive.setTargetPosition(elevatorDrive.getCurrentPosition());
                    elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    elevatorDrive.setPower(upPower - downPower);
                }
            }

            //Adjust Elevator Maximum Power
            if (elevatorMaxPowerUp && elevPowerUpDeb.milliseconds() > maxPowerToggleTimer) {//Change max. power that elevator motor will run at
                elevatorMaxPower += 0.1;
                elevPowerUpDeb.reset();
            } else if (elevatorMaxPowerDown && elevPowerDownDeb.milliseconds() < maxPowerToggleTimer) {//TODO: Toggle timers are magic numbers
                elevatorMaxPower -= 0.1;
                elevPowerDownDeb.reset();

            //Tiers
            } else if (elevatorTierUp && elevatorTier < 2 && elevTierUpDeb.milliseconds() > elevatorTierToggleTimer) {
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorDrive.setTargetPosition(elevatorDrive.getCurrentPosition());
                elevatorTier += 1;
            } else if (elevatorTierDown && elevatorTier > 0 && elevTierDownDeb.milliseconds() > elevatorTierToggleTimer) {
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorDrive.setTargetPosition(elevatorDrive.getCurrentPosition());
                elevatorTier -= 1;
                if (elevatorTier == 0) {
                    elevatorDrive.setTargetPosition(0);//Code below only runs when tier > 0, so it never turns target back to 0 when tier == 0!
                }

            //Set current encoder position to 0
            } else if (elevatorEncoderReset) {
                elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorDrive.setTargetPosition(0);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (elevatorDrive.getMode() == DcMotor.RunMode.RUN_TO_POSITION && elevatorDrive.getCurrentPosition() == elevatorDrive.getTargetPosition()) {
                //TODO: Now I'm checking this twice!
                status = "Redundant Check Returned Power";
                elevatorDrive.setPower(0);//If elevator is not being commanded, make sure it's stopped.
            }

            //Actually make the motor go to its designated tier (preliminary code...)
            if (elevatorTier != 0 && !elevatorDrive.isBusy()) {
                dtemp = elevatorTier * tierMultiplier;
                temp = (int) dtemp;
                elevatorDrive.setPower(0.2);
                status = "Set Tier Power";
                elevatorDrive.setTargetPosition(temp);
            } if (!elevatorDrive.isBusy()) {
                elevatorDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (temp-15 < elevatorDrive.getCurrentPosition() && elevatorDrive.getCurrentPosition() < temp+15) {
                elevatorDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

            //Claw Code
            if (clawToggle && clawDeb.milliseconds() > manipulatorToggleTimer) {
                if (clawFlag) {
                    clawServo.setPosition(clawClosedPosition);//Closes claw
                } else {
                    clawServo.setPosition(clawOpenPosition);
                }
                clawFlag = !clawFlag;
                clawDeb.reset();
            }

            //Arm Code
            if (armToggle && armDeb.milliseconds() > manipulatorToggleTimer) {
                if (armFlag) {
                    armServo.setPosition(armLeftPosition);
                } else {
                    armServo.setPosition(armRightPosition);
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
            if (forwardSlow) {
                forwardAxis = forwardSlowPower;
            } else if (forwardFull) {
                forwardAxis = forwardFullPower;
            }
            if (reverseSlow) {
                reverseAxis = reverseSlowPower;
            } else if (reverseFull) {
                reverseAxis = reverseFullPower;
            }

            if (turnLeftFull && turnRightFull) {
                turnPower = 0;
            } else if (turnLeftFull && !turnRightFull) {
                turnPower = turnLeftFullPower;
            } else if (turnRightFull && !turnLeftFull) {
                turnPower = turnRightFullPower;
            } else if (turnAxis != 0) {
                turnPower = turnAxis;
            } else if (turnAxis == 0 && turnLeftAxis == 0 && turnRightAxis == 0) {
                turnPower = 0;
            } else {
                turnAxis = turnRightAxis - turnLeftAxis;
            }

            if (strafeLeftFull && strafeRightFull) {
                strafePower = 0;
            } else if (strafeLeftFull) {
                strafePower = strafeLeftFullPower;//TODO: Add rather than set?
            } else if (strafeRightFull) {
                strafePower = strafeRightFullPower;
            } else if (strafeAxis != 0) {
                strafePower = strafeAxis;
            } else if (strafeAxis == 0 && strafeLeftAxis == 0 && strafeRightAxis == 0) {
                strafePower = 0;
            } else {
                strafeAxis = strafeRightAxis - strafeLeftAxis;
            }
            //TODO: Move these power sets closer to controls (or vice-versa)

            //Change Max Drive Power
            if (drivePowerUp && maxDrivePower < 1 && driveUpDeb.milliseconds() > maxPowerToggleTimer) {
                maxDrivePower += 0.1;
                driveUpDeb.reset();
            } else if (drivePowerDown && maxDrivePower > 0 && driveDownDeb.milliseconds() > maxPowerToggleTimer) {
                maxDrivePower -= 0.1;
                driveDownDeb.reset();
            }

            //Combine controls to allow a mix of all axes - kind of works?
            //TODO: As done for turn/strafePower, do for drivePower.
            leftPower    = Range.clip(forwardAxis + driveAxis + strafePower + turnPower - reverseAxis, -maxDrivePower, maxDrivePower) ;//Makes sure motors only run up to half power
            rightPower   = Range.clip(forwardAxis + driveAxis - strafePower - turnPower - reverseAxis, -maxDrivePower, maxDrivePower) ;//Adds all controller inputs together so they can kind of work simultaneously (it doesn't work very well right now)
            leftbackPower = Range.clip(forwardAxis + driveAxis - strafePower + turnPower - reverseAxis, -maxDrivePower, maxDrivePower) ;//I think it's +driveAxis on all of them?
            rightbackPower  = Range.clip(forwardAxis + driveAxis + strafePower - turnPower - reverseAxis , -maxDrivePower, maxDrivePower) ;

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
            telemetry.addData("Elevator Power", elevatorDrive.getPower());
            telemetry.addData("Elevator RunMode", elevatorDrive.getMode());
//            telemetry.addData("turnPower", turnPower);
//            telemetry.addData("strafePower", strafePower);
//            telemetry.addData("turnStick", gamepad1.left_stick_x);
//            telemetry.addData("strafeStick", gamepad1.right_stick_x);
            telemetry.update();
        }
        telemetry.addData("Status", "Stopped by User");
    }
}

