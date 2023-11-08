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
    ArmMotor armPivot;
    DcMotor armExtend;
    Servo clawServo;
    DcMotor dcMotorArmTemp;

    double upPower = 0;
    double downPower = 0;

    @Override
    public void init() {

        touch = hardwareMap.get(TouchSensor.class, "pivotLowerLimit");
        dcMotorArmTemp = hardwareMap.get(DcMotor.class, "armPivot");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        armPivot = new ArmMotor(dcMotorArmTemp);

        armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        armPivot.setTargetPosition(armPivot.getCurrentPosition());
        armPivot.setPower(0);
    }

    @Override
    public void loop() {
        armPivot.runToPosition();
    }
}
