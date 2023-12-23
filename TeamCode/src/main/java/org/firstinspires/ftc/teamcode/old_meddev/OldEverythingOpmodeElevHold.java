//ALoTO 2022-23
package org.firstinspires.ftc.teamcode.old_meddev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OldEverything Opmode (w/Auto Elev. Hold)", group="Everything Opmode")
//Start+A for driving, Start+B for manipulator
public class OldEverythingOpmodeElevHold extends LinearOpMode {
    private DcMotor elevatorDrive;
    private Servo servo1;//Claw Servo
    private Servo servo2;//Arm Servo
    //wheel motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private double maxElevPower = 0.5;//Change this to adjust the max elevator motor power, or use
    // the D-Pad to adjust it at runtime.
    private double maxDrivePower = 0.5;

    private int elevUpperLimit = 1060;
    private int elevLowerLimit = 0;

    @Override
    public void runOpMode() {
        elevatorDrive = hardwareMap.get(DcMotor.class, "Ellyvader");
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.FORWARD);
        //wheel motor names
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
         leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
          leftBack.setDirection(DcMotor.Direction.REVERSE);
         rightBack.setDirection(DcMotor.Direction.FORWARD);

        ServoController scont = servo1.getController();
        scont.pwmEnable();
        //ElapsedTime adeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);//Creating basically
        // a stopwatch that runs in the background
        //ElapsedTime xdeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime ddeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        //boolean aflag = true;
        //boolean xflag = true;
        //servo1.setPosition(0);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");

//Elevator code
        while (opModeIsActive()) {
            double up = gamepad2.right_trigger;
            double down = gamepad2.left_trigger;
            double upPower = Range.clip(up, 0, maxElevPower);//Makes sure elevator motor never runs above max. power
            double downPower = (-Range.clip(down, 0, maxElevPower))/2;
            telemetry.addData("Max Elevator Power", maxElevPower);
            telemetry.addData("Max Drive Power", maxDrivePower);
            if (up > 0 && elevatorDrive.getCurrentPosition() < elevUpperLimit) {
                elevatorDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorDrive.setPower(upPower);
            } else if (down > 0 && elevatorDrive.getCurrentPosition() > elevLowerLimit) {
                elevatorDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorDrive.setPower(downPower);
            } else if (gamepad2.dpad_up && maxElevPower < 0.9 && ddeb.milliseconds() > 100) {//Change max. power that elevator motor will run at
                maxElevPower += 0.1;
                ddeb.reset();
            } else if (gamepad2.dpad_down && maxElevPower > 0.1 && ddeb.milliseconds() > 100) {
                maxElevPower -= 0.1;
                ddeb.reset();
            } else if (gamepad2.dpad_right && ddeb.milliseconds() > 100) {
                elevUpperLimit += 10;
                ddeb.reset();
            } else if (gamepad2.dpad_left && ddeb.milliseconds() > 100) {
                elevUpperLimit -= 10;
                ddeb.reset();
            } else if (gamepad1.dpad_up && maxDrivePower < 0.9 && ddeb.milliseconds() > 100) {
                maxDrivePower += 0.1;
                ddeb.reset();
            } else if (gamepad1.dpad_down && maxDrivePower > 0.1 && ddeb.milliseconds() > 100) {
                maxDrivePower -= 0.1;
                ddeb.reset();
            } else if (gamepad2.back) {
                elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorDrive.setTargetPosition(0);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                elevatorDrive.setTargetPosition(elevatorDrive.getCurrentPosition());
                elevatorDrive.setPower(0.2);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //Claw Code
//            if (gamepad2.a && adeb.milliseconds() > 500) {
//                if (aflag) {
//                    servo1.setPosition(.26);//Closes claw
//                } else {
//                    servo1.setPosition(0);
//                }
//                aflag = !aflag;
//                adeb.reset();
//            } else if (gamepad2.x) {
//                servo1.setPosition(0);
//                aflag = false;
//            }
//            telemetry.addData("aflag", aflag);
//
//            //Arm Code
//            if (gamepad2.y && xdeb.milliseconds() > 500) {
//                if (xflag) {
//                    servo2.setPosition(0.6);
//                } else {
//                    servo2.setPosition(0);//Sends arm all the way left.
//                }
//            } else if (gamepad2.b) {
//                servo2.setPosition(0.6);
//                xflag = false;
//            }

            if (gamepad2.x) {
                servo1.setPosition(0.05);//Opens claw
            } else if (gamepad2.a) {
                servo1.setPosition(0.25);//Closes claw
            } else if (gamepad2.y && elevatorDrive.getCurrentPosition() > 200) {
                servo2.setPosition(0.07);
            } else if (gamepad2.b && elevatorDrive.getCurrentPosition() > 200) {
                servo2.setPosition(0.75);
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

            //Half-speed Driving
            if (gamepad1.right_bumper) {
                drive = 0.25;
            }
            if (gamepad1.left_bumper) {
                negative = 0.25;
            }

                 leftPower = Range.clip(drive + strafe + Range.clip(turn, -0.3, 0.3) - negative, -maxDrivePower, maxDrivePower) ;//Limits drive motors to half power
                rightPower = Range.clip(drive - strafe - Range.clip(turn, -0.3, 0.3) - negative, -maxDrivePower, maxDrivePower) ;//Sums controller inputs to combine them (works poorly)
             leftbackPower = Range.clip(drive - strafe + Range.clip(turn, -0.3, 0.3) - negative, -maxDrivePower, maxDrivePower) ;
            rightbackPower = Range.clip(drive + strafe - Range.clip(turn, -0.3, 0.3) - negative , -maxDrivePower, maxDrivePower) ;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftbackPower);
            rightBack.setPower(rightbackPower);

            telemetry.addData("upPower", upPower);
            telemetry.addData("downPower", downPower);
            telemetry.addData("targetPosition", elevatorDrive.getTargetPosition());
            telemetry.addData("currentPosition", elevatorDrive.getCurrentPosition());
            telemetry.addData("clawPosition", servo1.getPosition());
            telemetry.addData("armPosition", servo2.getPosition());
            telemetry.addData("upperLimit", elevUpperLimit);

            telemetry.update();
        }
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
