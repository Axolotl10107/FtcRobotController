package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ManipulatorTwo", group="TeleTest")
public class ManipulatorTwo extends OpMode {
    TouchSensor touch;
    DcMotor armPivot;
    DcMotor armExtend;
    Servo clawServo;

    double upPower = 0;
    double downPower = 0;

    @Override
    public void init() {
        touch = hardwareMap.get(TouchSensor.class, "pivotLowerLimit");
        armPivot = hardwareMap.get(DcMotor.class, "armPivot");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        armExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        armPivot.setTargetPosition(armPivot.getCurrentPosition());
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armPivot.setPower(0);
    }

    @Override
    public void loop() {
        telemetry.addData("Pressed", touch.isPressed());
//        if (0 < armExtend.getCurrentPosition() && armExtend.getCurrentPosition() < 2000) {
//            armExtend.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * 0.4);
//        }
        if (0 < armExtend.getCurrentPosition()) {
            downPower = gamepad2.left_trigger;
        } else {
            downPower = 0;
        }

        if (armExtend.getCurrentPosition() < 2000) {
            upPower = gamepad2.right_trigger;
        } else {
            upPower = 0;
        }

        armExtend.setPower((upPower - downPower) * 0.6);

        //claw
        if (gamepad2.x) {
            clawServo.setPosition(0.05);//Opens claw
        } else if (gamepad2.a) {
            clawServo.setPosition(0.19);//Closes claw
        }

        telemetry.addData("Encoder Position", armExtend.getCurrentPosition());
    }
}
