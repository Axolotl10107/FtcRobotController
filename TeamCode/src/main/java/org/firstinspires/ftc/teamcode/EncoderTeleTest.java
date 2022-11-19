package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="EncoderTeleTest", group="TeleTest")
public class EncoderTeleTest extends LinearOpMode {
    //Declare variables first because we have to
    private DcMotor motor;

    @Override
    public void runOpMode() {
        //Get the motor specified in the config file and set it up
        motor = hardwareMap.get(DcMotor.class, "LeftFront");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Wait for the PLAY button to be pressed (init has already happened!)
        waitForStart();

        //While the driver hasn't pressed the STOP button yet...
        while(opModeIsActive()) {
            //Send some info back to the driver
            telemetry.addData("Encoder reading", motor.getCurrentPosition());
            telemetry.addData("Target position", motor.getTargetPosition());
            telemetry.addData("Motor runmode", motor.getMode());

            motor.setPower(0.2);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int targetPos = 0;//Store our target position, so we can move only when we command it.
            //We can move an arbitrary amount of ticks this way.

            if (gamepad1.dpad_up) {//If UP on the D-Pad is pressed...
                targetPos += 1;//Increase the target position by 1 tick
            } else if (gamepad1.dpad_down) {
                targetPos -= 1;
            } else if (gamepad1.dpad_right) {
                targetPos += 10;//Now increase by 10 ticks
            } else if (gamepad1.dpad_left) {
                targetPos -= 10;
            } else if (gamepad1.a) {
                motor.setTargetPosition(targetPos);//Actually tell the motor to move to our stored target position
            } else if (gamepad1.x) {
                targetPos = 0;//Return motor to position 0
                motor.setTargetPosition(0);
            } else if (gamepad1.b) {
                motor.setPower(0);//Safety shutoff
            } else if (gamepad1.y) {
                motor.setPower(0.2);
            }
            /* Setting the target position will make the motor move immediately at the power level
            * we set earlier. It will go until it reaches that position, then it will stop. */

            //Transmit the telemetry back
            telemetry.update();
        }
    }
}
