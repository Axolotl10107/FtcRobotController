package org.firstinspires.ftc.teamcode.fy22;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class Robot {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public DcMotor elevatorDrive;
    public Servo armServo;
    public Servo clawServo;

    public DistanceSensor leftOdo;
    public DistanceSensor centerOdo;
    public DistanceSensor rightOdo;


    private int leftFrontTarget = 0;
    private int rightFrontTarget = 0;
    private int leftBackTarget = 0;
    private int rightBackTarget = 0;

    private int elevatorDriveTarget = 0;
    private boolean armServoTarget = false;
    private boolean clawServoTarget = false;

    private final float CPI = 100;//Unknown for now

    private void initialize() {
        elevatorDrive.setTargetPosition(elevatorDriveTarget);
        elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorDrive.setPower(0.5);
    }

    private void setDriveTarget(int forward, int right) {
        leftFrontTarget += forward;
        rightFrontTarget += forward;
        leftBackTarget += forward;
        rightBackTarget += forward;

        leftFrontTarget += right;
        rightFrontTarget -= right;
        leftBackTarget -= right;
        rightBackTarget += right;
    }
    public void intMotors(){
//        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
//        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
//        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
//        rightBack = hardwareMap.get(DcMotor.class, "RightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
    }

    public void encoder(float inches) {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition((int) (inches*CPI));
        rightFront.setTargetPosition((int) (inches*CPI));
        leftBack.setTargetPosition((int) (inches*CPI));
        rightBack.setTargetPosition((int) (inches*CPI));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);

        while (leftFront.isBusy()) {
            //TODO: PID
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setElevatorTarget(int target) {
        elevatorDrive.setTargetPosition(target);
    }
}
