//Michael's test program for running a motor to encoder positions
//Now with safety!

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
    ElapsedTime upDeb;
    ElapsedTime downDeb;
    ElapsedTime otherDeb;
    int upDebTime = 200; //waiting time in milliseconds
    int downDebTime = 200;
    int otherDebTime = 200;

    ElapsedTime safetyCheck;
    int safetyCheckTime = 100;

    ArrayList<String> motorList = new ArrayList<String>(6);

    int targetPosA = 0;//Stage target here - we'll send it to the motor later
    int targetPosB = 0;
    boolean aActive = true;
    boolean bActive = false;
    int listIdx = 0;
    double motorPower = 0.4;
    int lastMotorPos = 0;

    void initMotor(int idx) {
        String motorString = motorList.get(idx);
        telemetry.addData("Task", "Initializing motor...");
        motor = hardwareMap.get(DcMotor.class, motorString);

        //Put the motor into a known configuration
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(0);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//Set our current position as 0
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Task", "Done initializing motor");
    }

    void changeStagedTarget(int change) {
        if (aActive) {
            targetPosA += change;
        } else {
            targetPosB += change;
        }
    }

    void setStagedTarget(int value) {
        if (aActive) {
            targetPosA = value;
        } else {
            targetPosB = value;
        }
    }

    int getStagedTarget() {
        if (aActive) {
            return targetPosA;
        } else {
            return targetPosB;
        }
    }

    @Override
    public void init() {
        telemetry.addData("Task", "Initializing program...");
        //Add entries to motorList
        motorList.add("leftFront");
        motorList.add("leftBack");
        motorList.add("rightFront");
        motorList.add("rightBack");
        motorList.add("armPivot");
        motorList.add("armExtend");

        //Initialize the first motor
        initMotor(listIdx); //listIdx should be 0 at this time

        upDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        downDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        otherDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        safetyCheck = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addData("Task", "Ready");
    }

    @Override
    public void loop() {
        if (aActive) {
            telemetry.addData(">>> Staged Target A", targetPosA);
        } else {
            telemetry.addData("    Staged Target A", targetPosA);
        }
        if (bActive) {
            telemetry.addData(">>> Staged Target B", targetPosB);
        } else {
            telemetry.addData("    Staged Target B", targetPosB);
        }
        telemetry.addData("Active Target", motor.getTargetPosition());
        telemetry.addData("Actual Position", motor.getCurrentPosition());
        telemetry.addData("Motor runmode", motor.getMode());
        telemetry.addData("Motor power", motor.getPower());
        telemetry.addLine("------------------------------------------");
        telemetry.addLine("D-Pad: Up/Down 10, Left/Right 100");
        telemetry.addLine("Bumpers: 1000");
        telemetry.addLine("A/B set Active Target to respective Staged Target");
        telemetry.addLine("X stops motor");
        telemetry.addLine("Y sets current position as 0");
        telemetry.addLine("Right trigger toggles selected Staged Target");
        telemetry.addLine("Left trigger sets selected Staged Target to current position");
        telemetry.addLine("Start/Back cycle through motors");
        telemetry.addLine("Left/Right stick click change motor power");

        //Safety Check
        if (safetyCheck.milliseconds() > safetyCheckTime) {
            if (!(motor.getTargetPosition()-20 < motor.getCurrentPosition() && motor.getCurrentPosition() < motor.getTargetPosition()+20)) {
                //If we are not at our target position, then...
                int diff = motor.getCurrentPosition() - getStagedTarget();
                if (!(diff > 100 || diff < 100)) { //Both ways - could be moving either direction
                    motor.setPower(0);
                    motor.setTargetPosition(motor.getCurrentPosition());
                    telemetry.addData("Task", "Safety check tripped");
                }
            }
        }

        //Change Staged Target
        else if (gamepad1.right_trigger > 0.5 && otherDeb.milliseconds() > otherDebTime) {
            aActive = !aActive;
            bActive = !bActive;
            otherDeb.reset();
        }

        //Set Staged Target to Current Position
        else if (gamepad1.left_trigger > 0.5 && otherDeb.milliseconds() > otherDebTime) {
            setStagedTarget(motor.getCurrentPosition());
            otherDeb.reset();
        }

        //Motor power adjustment
        else if (gamepad1.right_stick_button && motor.getPower() < 1.0 && otherDeb.milliseconds() > otherDebTime) {
            motor.setPower(motor.getPower()+0.1);
            otherDeb.reset();
        } else if (gamepad1.left_stick_button && motor.getPower() > 0 && otherDeb.milliseconds() > otherDebTime) {
            motor.setPower(motor.getPower()-0.1);
            otherDeb.reset();
        }

        //Cycle selected motor
        else if (gamepad1.start && otherDeb.milliseconds() > otherDebTime) {
            if (listIdx == (motorList.size() - 1)) {
                listIdx = 0;
            } else {
                listIdx += 1;
            }
            otherDeb.reset();
        } else if (gamepad1.back && otherDeb.milliseconds() > otherDebTime) {
            if (listIdx == 0) {
                listIdx = (motorList.size() - 1);
            } else {
                listIdx -= 1;
            }
            otherDeb.reset();
        }

        //10-tick adjustment
        else if (gamepad1.dpad_up && upDeb.milliseconds() > upDebTime) {
            changeStagedTarget(10);
            upDeb.reset();
        } else if (gamepad1.dpad_down && downDeb.milliseconds() > downDebTime) {
            changeStagedTarget(-10);
            downDeb.reset();

        //100-tick adjustment
        } else if (gamepad1.dpad_right && upDeb.milliseconds() > upDebTime) {
            changeStagedTarget(100);
            upDeb.reset();
        } else if (gamepad1.dpad_left && downDeb.milliseconds() > downDebTime) {
            changeStagedTarget(-100);
            downDeb.reset();

        //1000-tick adjustment
        } else if (gamepad1.right_bumper && upDeb.milliseconds() > upDebTime) {
            changeStagedTarget(1000);
            upDeb.reset();
        } else if (gamepad1.left_bumper && downDeb.milliseconds() > downDebTime) {
            changeStagedTarget(-1000);
            downDeb.reset();
        }

        //Set the Active Target to Staged Target A
        else if (gamepad1.a && otherDeb.milliseconds() > otherDebTime) {
            motor.setPower(motorPower);
            motor.setTargetPosition(targetPosA);
            otherDeb.reset();
            telemetry.addData("Task", "Ready");

        //Set the Active Target to Staged Target B
        } else if (gamepad1.b && otherDeb.milliseconds() > otherDebTime) {
            motor.setPower(motorPower);
            motor.setTargetPosition(targetPosB);
            otherDeb.reset();
            telemetry.addData("Task", "Ready");

        //Stop Motor
        } else if (gamepad1.x && otherDeb.milliseconds() > otherDebTime) {
            motor.setPower(0);
            otherDeb.reset();
            telemetry.addData("Task", "Motor stopped");

        //Reset encoder position (set current position as 0)
        } else if (gamepad1.y && otherDeb.milliseconds() > otherDebTime) {
            telemetry.addData("Task", "Resetting encoder position...");
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            otherDeb.reset();
            telemetry.addData("Task", "Ready");
        }

    }
}
