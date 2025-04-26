package org.firstinspires.ftc.teamcode.fy23.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled //lacking important safety features
@TeleOp(name="backupCode Opmode", group="backupCode Opmode")
public class BackupCode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Arm = null; //arm
    private DcMotor Slides = null; //slides
    private Servo Claw = null; //claw
    //wheels
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        Claw = hardwareMap.get(Servo.class, "Claw");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        Claw.setDirection(Servo.Direction.FORWARD);

        //claw code
        if (gamepad2.x) {
            Claw.setPosition(0.05);//Opens claw
            telemetry.addData("Button", "X");
        } else if (gamepad2.a) {
            Claw.setPosition(.19);//Closes claw
            telemetry.addData("Button", "A");
        }
        //driving code
        double leftPower;
        double rightPower;
        double leftbackPower;
        double rightbackPower;

        double drive = gamepad1.right_trigger;
        double negative = gamepad1.left_trigger;
        double strafe  =  gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        double maxDrivePower = 1;

        leftPower = Range.clip(drive + strafe + turn - negative, -maxDrivePower, maxDrivePower) ;//Makes sure motors only run up to half power
        rightPower = Range.clip(drive - strafe - turn - negative, -maxDrivePower, maxDrivePower) ;//Adds all controller inputs together so they can kind of work simultaneously (it doesn't work very well right now)
        leftbackPower = Range.clip(drive - strafe + turn - negative, -maxDrivePower, maxDrivePower) ;
        rightbackPower = Range.clip(drive + strafe - turn - negative , -maxDrivePower, maxDrivePower) ;

        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        leftBack.setPower(leftbackPower);
        rightBack.setPower(rightbackPower);


    }
}
