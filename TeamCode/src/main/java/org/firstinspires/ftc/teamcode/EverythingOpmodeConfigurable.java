//ALoTO 2022-23
package org.firstinspires.ftc.teamcode;

//TODO: Elevator very broken (going up just jitters, tiers do nothing)
//TODO: Elevator max is 1000 now with new rigging!

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
    private double turnAxis;
    private boolean turnLeftFull;
    private static final double turnLeftFullPower = -1.0;
    private boolean turnRightFull;
    private static final double turnRightFullPower = 1.0;
    private double turnLeftAxis;
    private double turnRightAxis;
    private double strafeAxis;
    private boolean strafeLeftFull;
    private static final double strafeLeftFullPower = -1.0;
    private boolean strafeRightFull;
    private static final double strafeRightFullPower = 1.0;
    private double strafeLeftAxis;
    private double strafeRightAxis;
    private boolean clawToggle;
    private boolean armToggle;
    private static final int manipulatorToggleTimer = 500;
    private double elevatorAxis;
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

    //Claw and arm positions
    private static final double clawClosedPosition = 0.26;
    private static final double clawOpenPosition = 0;
    private static final double armLeftPosition = 0;// TODO: Left/right could be wrong, untested
    private static final double armRightPosition = 0.6;

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
        //New! Like elevator motor power, change drive motor power!
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
        armToggle = gamepad2.x;//boolean //NOTE: Could become a double in the future, for analog
        //arm position rather than an either-or situation

        //Elevator
        elevatorAxis = 0;//double //UNTESTED
        elevatorUpFull = false;//boolean //UNTESTED
        elevatorDownFull = false;//boolean //UNTESTED
        elevatorUpAxis = gamepad2.right_trigger;//double
        elevatorDownAxis = gamepad2.left_trigger;//double
        elevatorHold = gamepad2.b;//boolean //NOTE: Rarely Used
        elevatorMaxPowerUp = gamepad2.dpad_up;//boolean
        elevatorMaxPowerDown = gamepad2.dpad_down;//boolean
        //More Elevator - stuff that doesn't directly move it
        elevatorTierUp = gamepad2.right_bumper;//boolean //UNTESTED
        elevatorTierDown = gamepad2.left_bumper;//boolean //UNTESTED
        elevatorEncoderReset = gamepad2.back;//boolean //UNTESTED
    }

    @Override
    public void runOpMode() {
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

            setControls();

            if (elevatorAxis != 0) {
                elevatorPower = elevatorAxis;
            } else {
                double upPower = Range.clip(elevatorUpAxis, 0, elevatorMaxPower);//Makes sure elevator motor never runs above max. power
                double downPower = Range.clip(elevatorDownAxis, 0, elevatorMaxPower) / 2;//Divide by 2, because elevator is assisted by gravity and fast descent has broken it before!

                //Set elevator motor to power calculated above
//            if (elevatorUpAxis > 0) {
//                elevatorDrive.setPower(upPower);
//            }
                if (elevatorHold) {//Hold elevator up against gravity (largely unused now)
                    elevatorDrive.setPower(elevatorHoldPower);
                } else if (elevatorUpFull) {
                    elevatorDrive.setPower(elevatorUpFullPower);
                } else if (elevatorDownFull) {
                    elevatorDrive.setPower(elevatorDownFullPower);
                } else {
                    elevatorPower = upPower - downPower;
                }
//            } else if (elevatorDownAxis > 0) {
//                elevatorDrive.setPower(downPower);
//            }
            }
            elevatorDrive.setPower(elevatorPower);
            //Adjust Elevator Maximum Power
            if (elevatorMaxPowerUp && elevPowerUpDeb.milliseconds() > maxPowerToggleTimer) {//Change max. power that elevator motor will run at
                elevatorMaxPower += 0.1;
                elevPowerUpDeb.reset();
            } else if (elevatorMaxPowerDown && elevPowerDownDeb.milliseconds() < maxPowerToggleTimer) {//TODO: Toggle timers are magic numbers
                elevatorMaxPower -= 0.1;
                elevPowerDownDeb.reset();

                //Tiers
            } else if (elevatorTierUp && elevatorTier < 2 && elevTierUpDeb.milliseconds() > elevatorTierToggleTimer) {
                elevatorTier += 1;
            } else if (elevatorTierDown && elevatorTier > 0 && elevTierDownDeb.milliseconds() > elevatorTierToggleTimer) {
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
                int temp = (int) dtemp;
                elevatorDrive.setTargetPosition(temp);
            }

            //Claw Code
            if (clawToggle && clawDeb.milliseconds() > manipulatorToggleTimer) {
                if (clawFlag) {
                    servo1.setPosition(clawClosedPosition);//Closes claw
                } else {
                    servo1.setPosition(clawOpenPosition);
                }
                clawFlag = !clawFlag;
                clawDeb.reset();
            }

            //Arm Code
            if (armToggle && armDeb.milliseconds() > manipulatorToggleTimer) {
                if (armFlag) {
                    servo2.setPosition(armLeftPosition);
                } else {
                    servo2.setPosition(armRightPosition);
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

            if (turnLeftFull && !turnRightFull) {
                turnAxis = turnLeftFullPower;
            } else if (turnRightFull && !turnLeftFull) {
                turnAxis = turnRightFullPower;
            }

            if (strafeLeftFull && strafeRightFull) {
                strafeAxis = 0;
            } else if (strafeLeftFull) {
                strafeAxis = strafeLeftFullPower;//TODO: Add rather than set?
            } else if (strafeRightFull) {
                strafeAxis = strafeRightFullPower;
            }//TODO: Else, set to axis
            //TODO: Move these power sets closer to controls (or vice-versa)

            //Change Max Drive Power
            if (drivePowerUp && maxDrivePower < 1 && driveUpDeb.milliseconds() > maxPowerToggleTimer) {
                maxDrivePower += 0.1;
                driveUpDeb.reset();
            } else if (drivePowerDown && maxDrivePower > 0 && driveDownDeb.milliseconds() < maxPowerToggleTimer) {
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

