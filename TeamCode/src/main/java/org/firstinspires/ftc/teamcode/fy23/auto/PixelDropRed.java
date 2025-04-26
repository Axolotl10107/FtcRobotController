package org.firstinspires.ftc.teamcode.fy23.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="PixelDropRed", group="Auto Test")
public class PixelDropRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //    private ArmMotor armPivot = null;
    private DcMotor armPivot = null;
    private DcMotor armExtend = null;
    //motors for driving
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    static final double strafeSpeed = -1;


    @Override
    public void runOpMode() throws InterruptedException {
        // What to Accomplish in this code ;
        // * Extend the arm to a certain point
        // * Drive Left( Also in Auto code)
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        armPivot = hardwareMap.get(DcMotor.class, "armPivot");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);


        armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armPivot.setTargetPosition(armPivot.getCurrentPosition());
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setTargetPosition(armExtend.getCurrentPosition());
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();



        leftFront.setPower(0.08);
        leftBack.setPower(0.08);
        rightBack.setPower(0.08);
        rightFront.setPower(0.08);

        sleep(500);

        armPivot.setTargetPosition(armPivot.getCurrentPosition() + 1000);

        runtime.reset();
        armExtend.setPower(1);
        armPivot.setPower(0.5);

        sleep(1000);


//      i am deeply ashamed of what ive done

        sleep(100);
        leftFront.setPower(-strafeSpeed);
        rightFront.setPower(strafeSpeed);
        leftBack.setPower(strafeSpeed);
        rightBack.setPower(-strafeSpeed);


        // Code for Extending the Arm
        // Change this 0 to different number from trail and error.

        armExtend.setTargetPosition(1000);

        telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());

        telemetry.update();

        sleep(1000);

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);


        sleep(890);

        //Drive Code for red side pulling the robot out of teh board and then retracting the extenders.

        leftFront.setPower(strafeSpeed);
        rightFront.setPower(-strafeSpeed);
        leftBack.setPower(-strafeSpeed);
        rightBack.setPower(strafeSpeed);
        sleep(100);

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        
        //Retract the arm so that the manipulator opmode, it can default set the position of the arm to normal

        armExtend.setTargetPosition(0);

        sleep(1000);

        telemetry.addData("Path", "Completed");
        telemetry.update();

    }
}
