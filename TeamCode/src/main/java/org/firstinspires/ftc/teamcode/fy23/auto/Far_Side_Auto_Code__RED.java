package org.firstinspires.ftc.teamcode.fy23.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="AutoCodeFarSide__RED", group="AutoCodeFarSide__RED")
public class Far_Side_Auto_Code__RED extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // Motors for arm stuff
    private DcMotor armPivot = null;
    private DcMotor armExtend = null;
    //motors for driving
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    @Override
    public void runOpMode() {

        armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armPivot.setTargetPosition(armPivot.getCurrentPosition());
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setTargetPosition(armExtend.getCurrentPosition());
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armPivot.setTargetPosition(armPivot.getCurrentPosition() + 2000);
        waitForStart();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armPivot = hardwareMap.get(DcMotor.class, "armPivot");
        armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armPivot.setTargetPosition(armPivot.getCurrentPosition());

        telemetry.addData("Status", "Ready for Initialisation");



        //Code for Pivot the arm

        armPivot.setTargetPosition(500);
        armPivot.setPower(.4);
        sleep(200);

        armPivot.setPower(0.1);

        sleep(1000);

        //Code for moving forward

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);


        leftBack.setPower(.5);
        rightBack.setPower(.5);
        leftFront.setPower(.5);
        rightFront.setPower(.5);

        sleep(1600);

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        sleep(1800);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setPower(.63);
        rightBack.setPower(.5);
        leftFront.setPower(.63);
        rightFront.setPower(.5);

        sleep(3000);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setPower(.5);
        rightBack.setPower(.5);
        leftFront.setPower(.5);
        rightFront.setPower(.5);

        sleep(2000);









    }
}
