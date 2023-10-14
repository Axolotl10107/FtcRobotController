package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name= "DriveCode", group = "Linear opmode")
public class Drivecode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start and this is where the driver hits play
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;
            double leftbackPower;
            double rightbackPower;

            // POV Mode uses left stick to go forward, and right stick to strafe.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive;
            double negative;
            double strafe;
            double turn;

            boolean combinedTranslationControls = true;
            if (combinedTranslationControls) {
                drive = -gamepad1.right_stick_y*.75;
                negative = 0;
                strafe  =  gamepad1.right_stick_x*.75;
                turn = gamepad1.left_stick_x*.75;
            } else {
                drive = gamepad1.right_trigger;
                negative = gamepad1.left_trigger;
                strafe  =  gamepad1.right_stick_x;
                turn = gamepad1.left_stick_x;
            }




            leftPower    = Range.clip(drive + strafe + turn - negative, -2, 2) ;
            rightPower   = Range.clip(drive - strafe - turn - negative, -2, 2) ;
            leftbackPower = Range.clip(drive - strafe + turn - negative, -2, 2) ;
            rightbackPower  = Range.clip(drive + strafe - turn - negative , -2, 2) ;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftbackPower);
            rightBack.setPower(rightbackPower);


        }

        telemetry.update();
    }
}

