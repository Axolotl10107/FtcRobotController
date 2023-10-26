//Michael's test program for running a motor to encoder positions

package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="EncoderTeleTest23", group="TeleTest")
public class EncoderTeleTest23 extends OpMode {
    //Declare variables first because we have to
    DcMotor motor;

    ArrayList<String> motorList = new ArrayList<String>(6);

    int targetPosA = 0;//Stage target here - we'll send it to the motor later
    int targetPosB = 0;
    int tier = 0;
    double dtemp = 0;

    int listIdx = 0;

    void initMotor(int idx) {
        String motorString = motorList.get(idx);
        telemetry.addLine("Initializing motor...");
        motor = hardwareMap.get(DcMotor.class, motorString);

        //Put the motor into a known configuration
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(0);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//Set our current position as 0
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void init() {
        //Add entries to motorList
        motorList.add("leftFront");
        motorList.add("leftBack");
        motorList.add("rightFront");
        motorList.add("rightBack");
        motorList.add("armPivot");
        motorList.add("armExtend");

        //Get the first motor from the Hardware Map
        motor = hardwareMap.get(DcMotor.class, "Ellyvader");

        ElapsedTime upDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime downDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void loop() {
        telemetry.addData("Staged Target A", targetPosA);
        telemetry.addData("Staged Target B", targetPosB);
        telemetry.addData("Active Target", motor.getTargetPosition());
        telemetry.addData("Actual Position", motor.getCurrentPosition());
        telemetry.addData("Motor runmode", motor.getMode());

        //10-tick adjustment
        if (gamepad1.dpad_up) {
            targetPosA += 10;
        } else if (gamepad1.dpad_down) {
            targetPosA -= 10;

        //100-tick adjustment
        } else if (gamepad1.dpad_right) {
            targetPosA += 100;
        } else if (gamepad1.dpad_left) {
            targetPosA -= 100;

        //1000-tick adjustment
        } else if (gamepad1.right_bumper) {
            targetPosA += 1000;
        } else if (gamepad1.left_bumper) {
            targetPosA -= 1000;
        }

        //Move to Staged Target A
        else if (gamepad1.a) {
            motor.setPower(0.4);
            motor.setTargetPosition(targetPosA);

        //Move to Staged Target B
        } else if (gamepad1.b) {
            motor.setPower(0.4);
            motor.setTargetPosition(targetPosB);

        //Stop Motor
        } else if (gamepad1.x) {
            motor.setPower(0);

        //Reset encoder position (set current position as 0)
        } else if (gamepad1.y) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }
}
