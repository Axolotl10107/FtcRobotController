//Test program for running a motor to encoder positions
//Now with safety!
//This file is from July 24, 2024.

package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Iterator;

@TeleOp(name="EncoderTeleTest23", group="TeleTest")
public class EncoderTeleTest23 extends OpMode {

    //Configure the Safety Check
    ElapsedTime safetyCheckClock;
    int safetyPatience = 100; // in milliseconds, how long while the motor is not moving before stop
    double velocitySnapshot;
    boolean lockedOut = false;

    void safetyLockout(String trigger) {
        telemetry.clearAll();
        telemetry.addLine("Safety Lockout");
        telemetry.addLine("Press X to end the OpMode");
        telemetry.addData("Triggered by", trigger);
        telemetry.addLine("");
        telemetry.addData("Actual Motor Power", motor.getPower());
        telemetry.addData("Current Position", motor.getCurrentPosition());
        telemetry.addData("Active Target", motor.getTargetPosition());
        telemetry.addData("Current Velocity", motor.getVelocity());
        telemetry.addData("Snapshot when warning was triggered", velocitySnapshot);
        telemetry.addLine("Remember that I consider the absolute value.");
        telemetry.addLine("");
        telemetry.addData("hasPassedSafetyCheck", hasPassedSafetyCheck);
        telemetry.addData("SafetyCheckClock", safetyCheckClock.milliseconds());
        telemetry.addData("safetyPatience", safetyPatience);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otherDeb.reset();
        lockedOut = true;
    }

    //Declare variables first because we have to
    DcMotorEx motor;
    ElapsedTime upDeb;
    ElapsedTime downDeb;
    ElapsedTime otherDeb;
    int upDebTime = 200; //waiting time in milliseconds
    int downDebTime = 200;
    int otherDebTime = 200;

    ArrayList<DcMotorEx> motorList = new ArrayList<>(8); //A robot can have up to 8 motors.

    int targetPosA = 0; //Stage target here - we'll send it to the motor later
    int targetPosB = 0;
    boolean aActive = true;
    boolean bActive = false;
    int listIdx = 0;
    double motorPower = 0.4;

    boolean hasPassedSafetyCheck = false;

    void initMotor(int idx) {
        if (idx < motorList.size()) {
            motor = motorList.get(idx);
        }
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
        Iterator motorIterator = hardwareMap.dcMotor.iterator();
        while (motorIterator.hasNext()) {
            // *.next() just returns an Object. Here it's casted to a DcMotorEx.
            // (We're just making it the specific thing it actually is.)
            DcMotorEx tempMotor = (DcMotorEx) motorIterator.next();
            tempMotor.setPower(0.0);
            tempMotor.setTargetPosition(tempMotor.getCurrentPosition());
            tempMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorList.add(tempMotor);
        }

        initMotor(0); //Initialize the first motor

        upDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        downDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        otherDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        safetyCheckClock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addData("Task", "Ready");
    }

    @Override
    public void loop() {
        if (lockedOut) {
            telemetry.update();
            if (gamepad1.x) {
                requestOpModeStop(); //This method does nothing in virtual_robot.
            }
        } else {
            if (aActive) {
                telemetry.addData("->>>Staged Target A", targetPosA);
            } else {
                telemetry.addData("----Staged Target A", targetPosA);
            }
            if (bActive) {
                telemetry.addData("->>>Staged Target B", targetPosB);
            } else {
                telemetry.addData("----Staged Target B", targetPosB);
            }
            telemetry.addData("Active Target", motor.getTargetPosition());
            telemetry.addData("Actual Position", motor.getCurrentPosition());
            telemetry.addData("Active Motor", hardwareMap.getNamesOf(motor).iterator().next());
            telemetry.addData("Port Number", motor.getPortNumber());
            telemetry.addLine(motor.getDeviceName());
            telemetry.addData("Motor runmode", motor.getMode());
            telemetry.addData("Set Motor Power", motorPower);
            telemetry.addData("Actual Motor power", motor.getPower());
            telemetry.addData("Motor Velocity", motor.getVelocity());
            telemetry.addLine("");
            telemetry.addData("Safety Patience (ms)", safetyPatience);
            telemetry.addData("hasPassedSafetyCheck", hasPassedSafetyCheck);
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
            telemetry.addLine("Extra stuff on Gamepad 2:");
            telemetry.addLine("A sets RUN_WITHOUT_ENCODER");
            telemetry.addLine("B sets RUN_TO_POSITION");
            telemetry.addLine("Triggers do analog control when in RUN_WITHOUT_ENCODER");

            // Safety Check
            if (Math.abs(motor.getPower()) > 0.05) {
                if (Math.abs(motor.getCurrentPosition()) < Math.abs(motor.getTargetPosition()) - 50) {
                    if (!hasPassedSafetyCheck) {
                        if (safetyCheckClock.milliseconds() > safetyPatience) {
                            velocitySnapshot = motor.getVelocity();
                            safetyLockout("Patience exhausted (motor appears to have never started moving)");
                        } else if (Math.abs(motor.getVelocity()) > 1) {
                            hasPassedSafetyCheck = true;
                        }
                    } else if (hasPassedSafetyCheck) {
                        if (Math.abs(motor.getVelocity()) < 1) {
                            velocitySnapshot = motor.getVelocity();
                            safetyLockout("Velocity is less than 1 (motor appears to have stopped moving)");
                        }
                    }
                } else {
                    hasPassedSafetyCheck = false;
                }
            }

            //Change Staged Target
            if (gamepad1.right_trigger > 0.5 && otherDeb.milliseconds() > otherDebTime) {
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
            else if (gamepad1.right_stick_button && motorPower < 1.0 && otherDeb.milliseconds() > otherDebTime) {
                motorPower = (motorPower + 0.1);
                otherDeb.reset();
            } else if (gamepad1.left_stick_button && motorPower > 0.0 && otherDeb.milliseconds() > otherDebTime) {
                motorPower = (motorPower - 0.1);
                otherDeb.reset();
            }

            //Cycle selected motor
            else if (gamepad1.start && otherDeb.milliseconds() > otherDebTime) {
                if (listIdx == (motorList.size() - 1)) {
                    listIdx = 0;
                } else {
                    listIdx += 1;
                }
                initMotor(listIdx);
                otherDeb.reset();
            } else if (gamepad1.back && otherDeb.milliseconds() > otherDebTime) {
                if (listIdx == 0) {
                    listIdx = (motorList.size() - 1);
                } else {
                    listIdx -= 1;
                }
                initMotor(listIdx);
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
                safetyCheckClock.reset();
                telemetry.addData("Task", "Ready");

                //Set the Active Target to Staged Target B
            } else if (gamepad1.b && otherDeb.milliseconds() > otherDebTime) {
                motor.setPower(motorPower);
                motor.setTargetPosition(targetPosB);
                otherDeb.reset();
                safetyCheckClock.reset();
                telemetry.addData("Task", "Ready");

                //Stop Motor
            } else if (gamepad1.x && otherDeb.milliseconds() > otherDebTime) {
                motor.setPower(0);
                otherDeb.reset();
                telemetry.addData("Task", "Motor stopped");

                //Reset encoder position (set current position as 0)
            } else if (gamepad1.y && otherDeb.milliseconds() > otherDebTime) {
                telemetry.addData("Task", "Resetting encoder position...");
                motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                motor.setTargetPosition(0);
                motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                otherDeb.reset();
                telemetry.addData("Task", "Ready");

                //Bonus feature: analog control with gamepad2
            } else if (gamepad2.a && otherDeb.milliseconds() > otherDebTime) {
                motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                otherDeb.reset();
            } else if (gamepad2.b && otherDeb.milliseconds() > otherDebTime) {
                motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                otherDeb.reset();
            }

            if (motor.getMode() == DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
                motor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            }

        }
    }
}
