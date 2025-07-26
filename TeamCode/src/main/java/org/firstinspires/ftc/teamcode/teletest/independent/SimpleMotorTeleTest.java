package org.firstinspires.ftc.teamcode.teletest.independent;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.TriggerButton;

import java.util.ArrayList;
import java.util.Iterator;

@TeleOp()
public class SimpleMotorTeleTest extends OpMode {

    ArrayList<DcMotorEx> motorList = new ArrayList<>(8); //A robot can have up to 8 motors.
    DcMotorEx motor;

    TriggerButton nextMotorButton;
    TriggerButton prevMotorButton;
    TriggerButton powerUpButton;
    TriggerButton powerDownButton;
    TriggerButton stopButton;

    int motorNum = 0;

    void initMotor(int idx) {
        if (idx < motorList.size()) {
            motor = motorList.get(idx);
            motor.setPower(0.0);
        }
    }

    @Override
    public void init() {
        nextMotorButton = new TriggerButton(() -> gamepad1.start);
        prevMotorButton = new TriggerButton(() -> gamepad1.back);
        powerUpButton = new TriggerButton(() -> gamepad1.right_bumper);
        powerDownButton = new TriggerButton(() -> gamepad1.left_bumper);
        stopButton = new TriggerButton(() -> gamepad1.b);

        Iterator motorIterator = hardwareMap.dcMotor.iterator();
        while (motorIterator.hasNext()) {
            // *.next() just returns an Object. Here it's casted to a DcMotorEx.
            // (We're just making it the specific thing it actually is.)
            DcMotorEx tempMotor = (DcMotorEx) motorIterator.next();
            tempMotor.setPower(0.0);
            tempMotor.setTargetPosition(tempMotor.getCurrentPosition());
            tempMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorList.add(tempMotor);
        }

        initMotor(motorNum);
    }

    @Override
    public void loop() {
        telemetry.addData("Motor", hardwareMap.getNamesOf(motor).iterator().next());
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Encoder Position", motor.getCurrentPosition());
        telemetry.addData("RunMode", motor.getMode());

        if (powerUpButton.isActive()) {
            motor.setPower(motor.getPower() + 0.1);
        }

        if (powerDownButton.isActive()) {
            motor.setPower(motor.getPower() - 0.1);
        }

        if (stopButton.isActive()) {
            motor.setPower(0);
        }

        if (nextMotorButton.isActive() && motorNum < 7) {
            motorNum += 1;
            initMotor(motorNum);
        }

        if (prevMotorButton.isActive() && motorNum > 0) {
            motorNum -= 1;
            initMotor(motorNum);
        }
    }
}
