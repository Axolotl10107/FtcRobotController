package org.firstinspires.ftc.teamcode.fy23;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Bad Autonomous Blue", group="Autonomous")
public class BadAutoBlue extends LinearOpMode {

    private DcMotor leftFront= null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor armPivot = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double strafeSpeed = -1;

    @Override
    public void runOpMode() {

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


        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();




        runtime.reset();

//      i am deeply ashamed of what ive done
        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
            armPivot.setPower(.5);
            sleep(200);
            armPivot.setPower(0);
            leftFront.setPower(strafeSpeed);
            rightFront.setPower(-strafeSpeed);
            leftBack.setPower(-strafeSpeed);
            rightBack.setPower(strafeSpeed);

            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        telemetry.addData("Path", "Completed");
        telemetry.update();
        sleep(1000);


    }
}
