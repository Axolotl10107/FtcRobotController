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
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadInputs;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadLinear;


@TeleOp(name="Manipulator Opmode", group="Manipulator Opmode")
public class Manipulator_Code extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Arm = null;
    //motors for driving
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    @Override
    public void runOpMode() {
        GamepadLinear Controls = new GamepadLinear(gamepad1, gamepad2);

        telemetry.addData("Status", "Ready for Initialisation");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //code runs the arm up and down
        Arm = hardwareMap.get(DcMotor.class, "ArmPivot");
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Code for arm moving up and down

//            // This code moves the ArmMotor forward
//            double ArmMotorpower;
//
//            double MoveForward = -2;
//
//            boolean Anything = true;
//            if (Anything) {
//                MoveForward = gamepad1.right_trigger * .75;
//            }
//            ArmMotorpower = Range.clip(MoveForward, -2, 2);
//            Arm.setPower(ArmMotorpower);
//
//            // This code moves the ArmMotor backwards
//            double ArmMotorpowerback;
//
//            double MoveBackward = -2;
//
//            boolean AnythingBack = true;
//            if (AnythingBack) {
//                MoveBackward = gamepad1.left_trigger * -.75;
//            }
//            ArmMotorpowerback = Range.clip(MoveBackward, -2, 2);
//            Arm.setPower(ArmMotorpowerback);

            //Code for Arm moving out and in

             //This code moves the Arm out *TEST*

            // This code moves the Arm In *TEST*

            Arm.setPower(Controls.elevatorMovement());


            //Code for the claw *TEST*
            if (Controls.clawOpen() == 1) {
                servo1.setPosition(0.05);//Opens claw
            } else if (Controls.clawClose() == 1) {
                servo1.setPosition(0.19);//Closes claw
            }
            telemetry.update();

            leftFront.setPower(Controls.forwardMovement() + Controls.strafeMovement() + Controls.rotateMovement());
            rightFront.setPower(Controls.forwardMovement() - Controls.strafeMovement() - Controls.rotateMovement());
            leftBack.setPower(Controls.forwardMovement() - Controls.strafeMovement() + Controls.rotateMovement());
            rightBack.setPower(Controls.forwardMovement() + Controls.strafeMovement() - Controls.rotateMovement());

            //Drive code TEST

        }
    }
}
