package org.firstinspires.ftc.teamcode;

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

    private void setElevatorTarget(int target) {
        elevatorDrive.setTargetPosition(target);
    }
}
