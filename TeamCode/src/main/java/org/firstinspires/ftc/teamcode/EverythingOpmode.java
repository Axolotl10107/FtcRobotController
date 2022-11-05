//ALoTO 2022-23
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Everything Opmode", group="Linear Opmode")

public class ElevatorTeleTest extends LinearOpMode {
    private DcMotor elevatorDrive;
    private Servo servo1;
    private Servo servo2;
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
        //wheel motor names
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE );
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double up = gamepad2.right_trigger;
            double down = gamepad2.left_trigger;
            double upPower = Range.clip(up, 0, maxPower);
            double downPower = -Range.clip(down, 0, maxPower);
            telemetry.addData("        Up Trigger:", up);
            telemetry.addData("      Down Trigger:", down);
            telemetry.addData("         Max Power:", maxPower);
            telemetry.addData("  Clipped Up Power:", upPower);
            telemetry.addData("Clipped Down Power:", downPower);
            telemetry.addData(" Upper Limit State:", upperLimit.getState());
            telemetry.addData(" Lower Limit State:", lowerLimit.getState());
            if (up > 0 && upperLimit.getState() == false) {//Limit switches are normally closed.
                elevatorDrive.setPower(upPower);
            } else if (down > 0 && lowerLimit.getState()) {
                elevatorDrive.setPower(downPower);
            } else if (gamepad2.dpad_up) {
                maxPower += 0.1;
                sleep(100);
            } else if (gamepad2.dpad_down) {
                maxPower -= 0.1;
                sleep(100);
            } else if (gamepad2.left_bumper) {
                double startTime = getRuntime();
                elevatorDrive.setPower(-maxPower);
                while (lowerLimit.getState()) {
                    telemetry.addData("Running for (seconds):", getRuntime() - startTime);
                }
                elevatorDrive.setPower(0);
            } else if (gamepad2.right_bumper) {
                double startTime = getRuntime();
                elevatorDrive.setPower(maxPower);
                while (upperLimit.getState() == false) {
                    telemetry.addData("Running for (seconds):", getRuntime() - startTime);
                }
                elevatorDrive.setPower(0);
            } else {
                elevatorDrive.setPower(0);
            }
            servo1.setPosition((gamepad2.left_stick_x / 2) + 0.5);
            //+ 0.5 - stick goes -1 to 1, servo goes 0 to 1. This offsets the stick range.
            //Div. stick position by 2, so that, with the offset, 0 and 1 are at edges of stick

            servo2.setPosition((gamepad2.right_stick_x / 2) + 0.5);
            //driving code
            double leftPower;
            double rightPower;
            double leftbackPower;
            double rightbackPower;

            double drive = gamepad1.right_trigger;
            double negative = gamepad1.left_trigger;
            double strafe  =  gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_x;


            leftPower    = Range.clip(drive + strafe + turn - negative, -.5, .5) ;
            rightPower   = Range.clip(drive - strafe - turn - negative, -.5, .5) ;
            leftbackPower = Range.clip(drive - strafe + turn - negative, -.5, .5) ;
            rightbackPower  = Range.clip(drive + strafe - turn - negative , -.5, .5) ;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftbackPower);
            rightBack.setPower(rightbackPower);

            telemetry.update();
        }
    }
}

