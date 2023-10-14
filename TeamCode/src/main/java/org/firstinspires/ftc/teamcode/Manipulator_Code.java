package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // This code moves the ArmMotor forward
            double ArmMotorpower;

            double MoveForward = -2;

            boolean Anything = true;
                    if (Anything) {
                        MoveForward = gamepad1.right_trigger*.75;
                    }
            ArmMotorpower = Range.clip(MoveForward, -2, 2) ;
            Arm.setPower(ArmMotorpower);


            // This code moves the ArmMotor backwards
            double ArmMotorpowerback;

            double MoveBackward = -2;

            boolean AnythingBack = true;
            if (AnythingBack) {
                MoveBackward = gamepad1.left_trigger*-.75;
            }
            ArmMotorpowerback = Range.clip(MoveBackward, -2, 2) ;
            Arm.setPower(ArmMotorpowerback);

        }
          telemetry.update();
        }
    }
