package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

@TeleOp()
public class BIT extends LinearOpMode {

    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;

    DcMotorEx leftPivot;
    DcMotorEx rightPivot;

    DcMotorEx leftExtend;
    DcMotorEx rightExtend;

    boolean testMotor(DcMotorEx motor, String string) {
        telemetry.addLine(string);
        telemetry.update();

        motor.setPower(0.3);
        sleep(1000);
        motor.setPower(0);

        if (motor.getCurrentPosition() > 40) {
            telemetry.addLine(string + " PASS");
            telemetry.update();
            return true;
        } else {
            telemetry.addLine(string + " FAIL");
            telemetry.update();
            return false;
        }
    }

    ArrayList<Boolean> testTwoMotors(DcMotorEx motor1, String string1, DcMotorEx motor2, String string2) {
        telemetry.addLine("Two motors:");
        telemetry.addLine(string1);
        telemetry.addLine(string2);
        telemetry.update();

        motor1.setPower(0.3);
        motor2.setPower(0.3);
        sleep(1000);
        motor1.setPower(0);
        motor2.setPower(0);

        boolean motor1pass;
        boolean motor2pass;
        if (motor1.getCurrentPosition() > 40) {
            telemetry.addLine(string1 + " PASS");
            motor1pass = true;
        } else {
            telemetry.addLine(string1 + " FAIL");
            motor1pass = false;
        }
        if (motor2.getCurrentPosition() > 40) {
            telemetry.addLine(string2 + " PASS");
            motor2pass = true;
        } else {
            telemetry.addLine(string2 + " FAIL");
            motor2pass = false;
        }

        if (motor1.getCurrentPosition() > 40 && motor2.getCurrentPosition() > 40) {
            telemetry.addLine("System PASS");
        } else {
            telemetry.addLine("System FAIL");
        }

        telemetry.update();

        ArrayList<Boolean> returnList = new ArrayList<>();
        returnList.add(motor1pass);
        returnList.add(motor2pass);
        return returnList;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftPivot = hardwareMap.get(DcMotorEx.class, "armLeftPivot");
        leftPivot.setDirection(DcMotorEx.Direction.REVERSE);
        rightPivot = hardwareMap.get(DcMotorEx.class, "armRightPivot");

        leftExtend = hardwareMap.get(DcMotorEx.class, "armLeftExtend");
        rightExtend = hardwareMap.get(DcMotorEx.class, "armRightExtend");
        rightExtend.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        boolean leftFrontPass = testMotor(leftFront, "leftFront");
        sleep(1000);
        boolean rightFrontPass = testMotor(rightFront, "rightFront");
        sleep(1000);
        boolean leftBackPass = testMotor(leftBack, "leftBack");
        sleep(1000);
        boolean rightBackPass = testMotor(rightBack, "rightBack");
        sleep(1000);

        ArrayList<Boolean> pivotPass = testTwoMotors(leftPivot, "leftPivot", rightPivot, "rightPivot");
        sleep(1000);
        ArrayList<Boolean> extendPass = testTwoMotors(leftExtend, "leftExtend", rightExtend, "rightExtend");
        sleep(1000);


        telemetry.addLine("System Report");

        if (leftFrontPass && rightFrontPass && leftBackPass && rightBackPass) {
            telemetry.addLine("Drive System PASS");
        } else {
            telemetry.addLine("Drive System FAIL - more information:");
            if (!leftFrontPass) {
                telemetry.addLine("leftFront FAIL");
            }
            if (!rightFrontPass) {
                telemetry.addLine("rightFront FAIL");
            }
            if (!leftBackPass) {
                telemetry.addLine("leftBack FAIL");
            }
            if (!rightBackPass) {
                telemetry.addLine("rightBack FAIL");
            }
        }

        telemetry.addLine("----------------");

        if (pivotPass.get(1) && pivotPass.get(2)) {
            telemetry.addLine("Pivot System PASS");
        } else {
            telemetry.addLine("Pivot System FAIL - more information:");
            if (!pivotPass.get(1)) {
                telemetry.addLine("leftPivot FAIL");
            }
            if (!pivotPass.get(2)) {
                telemetry.addLine("rightPivot FAIL");
            }
        }

        telemetry.addLine("----------------");

        if (extendPass.get(1) && extendPass.get(2)) {
            telemetry.addLine("Extend System PASS");
        } else {
            telemetry.addLine("Extend System FAIL - more information:");
            if (!extendPass.get(1)) {
                telemetry.addLine("leftExtend FAIL");
            }
            if (!extendPass.get(2)) {
                telemetry.addLine("rightExtend FAIL");
            }
        }

        telemetry.update();
        while (true) {
            sleep(1000);
        }
    }
}
