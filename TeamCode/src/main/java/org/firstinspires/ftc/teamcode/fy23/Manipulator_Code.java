package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;


@TeleOp(name="Manipulator Opmode", group="Manipulator Opmode")
public class Manipulator_Code extends LinearOpMode {

    private DcMotor Arm = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Ready for Initialisation");
        //code runs the arm up and down
        Arm = hardwareMap.get(DcMotor.class, "ArmMotor");
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Code for arm moving up and down

            // This code moves the ArmMotor forward
            double ArmMotorpower;

            double MoveForward = -2;

            boolean Anything = true;
            if (Anything) {
                MoveForward = gamepad1.right_trigger * .75;
            }
            ArmMotorpower = Range.clip(MoveForward, -2, 2);
            Arm.setPower(ArmMotorpower);

            // This code moves the ArmMotor backwards
            double ArmMotorpowerback;

            double MoveBackward = -2;

            boolean AnythingBack = true;
            if (AnythingBack) {
                MoveBackward = gamepad1.left_trigger * -.75;
            }
            ArmMotorpowerback = Range.clip(MoveBackward, -2, 2);
            Arm.setPower(ArmMotorpowerback);

            //Code for Arm moving out and in

            // This code moves the Arm out *TEST*
            double ArmOutMotorPower;

            double MoveOut = 0;

            boolean AnythingOut = true;
            if (AnythingOut) {
                if (gamepad1.right_bumper) {
                    MoveOut = .75;
                }
//                MoveOut = gamepad1.right_trigger * .75;
            }
            ArmOutMotorPower = Range.clip(MoveOut, -2, 2);
            Arm.setPower(ArmOutMotorPower);

            // This code moves the Arm In *TEST*
            double ArmInMotorPower;

            double MoveIn = 0;

            boolean AnythingIn = true;
            if (AnythingIn) {
                if (gamepad1.left_bumper) {
                    MoveIn = -.75;
                }
//                MoveIn = gamepad1.left_trigger * -1.75;
            }
            ArmInMotorPower = Range.clip(MoveIn, -2, 2);
            Arm.setPower(ArmInMotorPower);


            //Code for the claw *TEST*
            if (gamepad1.x) {
                servo1.setPosition(0.05);//Opens claw
            } else if (gamepad1.a) {
                servo1.setPosition(0.19);//Closes claw
            }
            telemetry.update();
        }
    }
}
