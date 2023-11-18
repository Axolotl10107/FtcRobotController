package org.firstinspires.ftc.teamcode.fy23;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoFunctions {
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    private double circumference;
    private int ticksPerRotation;


    AutoFunctions(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, double circ, int tpr) {
        lf = leftFront;
        rf = rightFront;
        lb = leftBack;
        rb = rightBack;
        circumference = circ;
        ticksPerRotation = tpr;
    }

    public void moveForward(double power, double distance) {

    }
    public void strafeLeft(double power) {
        lf.setPower(power);
        rf.setPower(-power);
        lb.setPower(-power);
        rb.setPower(power);
    }


}
