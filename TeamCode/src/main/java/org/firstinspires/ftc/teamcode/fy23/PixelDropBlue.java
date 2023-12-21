package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="PixelDropBlue", group="Auto Test")
public class PixelDropBlue extends LinearOpMode {

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

        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setTargetPosition(armExtend.getCurrentPosition());

        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armPivot.setPower(0.4);

        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setPower(0.14);
        rightFront.setPower(0.14);

        sleep(1000);

        runtime.reset();
        armExtend.setPower(1);
        armPivot.setPower(1);
//      i am deeply ashamed of what ive done

        armPivot.setTargetPosition(1300);
        sleep(200);
        leftFront.setPower(strafeSpeed);
        rightFront.setPower(-strafeSpeed);
        leftBack.setPower(-strafeSpeed);
        rightBack.setPower(strafeSpeed);


        // Code for Extending the Arm
        // Change this 0 to different number from trail and error.

        armExtend.setTargetPosition(1350);

        telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());

        telemetry.update();

        sleep(1000);

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        sleep(890);

        leftFront.setPower(-strafeSpeed);
        rightFront.setPower(strafeSpeed);
        leftBack.setPower(strafeSpeed);
        rightBack.setPower(-strafeSpeed);

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);


        sleep(900);
        armExtend.setTargetPosition(0);

        telemetry.addData("Path", "Completed");
        telemetry.update();
        sleep(1000);

    }
}
