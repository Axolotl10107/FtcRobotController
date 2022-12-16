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

@TeleOp(name="Everything Opmode", group="Everything Opmode")
//Start+A for driving, Start+B for manipulator
public class EverythingOpmode extends LinearOpMode {
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

    @Override
    public void runOpMode() {
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
        ElapsedTime adeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean aflag = true;
        servo1.setPosition(0);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

//Elevator code
        while (opModeIsActive()) {
            double up = gamepad2.right_trigger;
            double down = gamepad2.left_trigger;
            double upPower = Range.clip(up, 0, maxPower);//Makes sure elevator motor never runs above max. power
            double downPower = (-Range.clip(down, 0, maxPower))/2;
            telemetry.addData("Max Elevator Power:", maxPower);
            //No limit switches currently installed!
//            if (up > 0 && upperLimit.getState() == false) {//Limit switches are normally closed.
            if (up > 0) {
                elevatorDrive.setPower(upPower);
//            } else if (down > 0 && lowerLimit.getState() == false) {
            }
            else if (gamepad2.right_bumper) {//Hold elevator up against gravity
                elevatorDrive.setPower(.2);
            }
            else if (down > 0) {
                elevatorDrive.setPower(downPower);
            } else if (gamepad2.dpad_up) {//Change max. power that elevator motor will run at
                maxPower += 0.1;
                sleep(100);
            } else if (gamepad2.dpad_down) {
                maxPower -= 0.1;
                sleep(100);
            }
            //Elevator auto up/down; again, no limit switches installed. This code will likely be
            //removed soon in favor of using encoders for 4 elevator tiers.
//            else if (gamepad2.left_bumper) {
//                double startTime = getRuntime();
//                elevatorDrive.setPower(-maxPower);
//                while (lowerLimit.getState()) {
//                    telemetry.addData("Running for (seconds):", getRuntime() - startTime);
//                }
//                elevatorDrive.setPower(0);
//            } else if (gamepad2.right_bumper) {
//                double startTime = getRuntime();
//                elevatorDrive.setPower(maxPower);
//                while (upperLimit.getState() == false) {
//                    telemetry.addData("Running for (seconds):", getRuntime() - startTime);
//                }
//                elevatorDrive.setPower(0);
//            }
            else {
                elevatorDrive.setPower(0);//If elevator is not being commanded, make sure it's stopped.
            }

            //Claw Code
            //servo1.setPosition((gamepad2.left_stick_x / 2) + 0.5);//Old absolute position code
            if (gamepad2.x) {
                // .45
                servo1.setPosition(0);//Opens claw
            }
            else if (gamepad2.a && adeb.milliseconds() > 500) {
                if (aflag) {
                    servo1.setPosition(.26);//Closes claw
                } else {
                    servo1.setPosition(0);
                }
                aflag = !aflag;
                adeb.reset();
            }
            telemetry.addData("aflag", aflag);
            //+ 0.5 - stick goes -1 to 1, servo goes 0 to 1. This offsets the stick range.
            //Div. stick position by 2, so that, with the offset, 0 and 1 are at edges of stick

            //Arm Code
            // servo2.setPosition((gamepad2.right_stick_x / 2) + 0.5);
            if (gamepad2.y) {
                servo2.setPosition(0);//Sends arm all the way [].
            }
            else if (gamepad2.b) {
                servo2.setPosition(.6);
            }
            //driving code
            double leftPower;
            double rightPower;
            double leftbackPower;
            double rightbackPower;

            double drive = gamepad1.right_trigger;
            double negative = gamepad1.left_trigger;
            double strafe  =  gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_x;


            leftPower    = Range.clip(drive + strafe + turn - negative, -.5, .5) ;//Makes sure motors only run up to half power
            rightPower   = Range.clip(drive - strafe - turn - negative, -.5, .5) ;//Adds all controller inputs together so they can kind of work simultaneously (it doesn't work very well right now)
            leftbackPower = Range.clip(drive - strafe + turn - negative, -.5, .5) ;
            rightbackPower  = Range.clip(drive + strafe - turn - negative , -.5, .5) ;

            //Half-speed Driving
            if (gamepad1.right_bumper) {
                drive = 0.25;
            }
            if (gamepad1.left_bumper) {
                negative = 0.25;
            }

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftbackPower);
            rightBack.setPower(rightbackPower);

            telemetry.update();
        }
    }
}
