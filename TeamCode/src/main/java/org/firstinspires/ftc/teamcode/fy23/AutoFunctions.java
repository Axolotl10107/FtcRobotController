package org.firstinspires.ftc.teamcode.fy23;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoFunctions {
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;

    AutoFunctions(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        lf = leftFront;
        rf = rightFront;
        lb = leftBack;
        rb = rightBack;
    }

    public void moveForward(double power) {
        lf.setPower(power);
        rf.setPower(power);
        lb.setPower(power);
        rb.setPower(power);
    }
    public void strafeLeft(double power) {
        lf.setPower(power);
        rf.setPower(-power);
        lb.setPower(-power);
        rb.setPower(power);
    }


}
