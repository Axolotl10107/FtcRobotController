package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoCodeFarSide__BLUE", group="AutoCodeFarSide__BLUE")
public class Far_Side_Auto_Code__BLUE extends LinearOpMode {

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

        waitForStart();

        telemetry.addData("Status", "Ready for Initialisation");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armPivot = hardwareMap.get(DcMotor.class, "armPivot");
        armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armPivot.setTargetPosition(armPivot.getCurrentPosition());

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

        sleep(1800);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setPower(.5);
        rightBack.setPower(.65);
        leftFront.setPower(.5);
        rightFront.setPower(.65);

        sleep(3800);






    }
}
