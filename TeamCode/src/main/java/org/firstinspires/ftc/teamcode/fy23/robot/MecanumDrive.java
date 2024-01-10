package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

public class MecanumDrive {

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public MecanumDrive(HardwareMap hardwareMap, String lfName, String rfName, String lbName, String rbName) {
        leftFront = hardwareMap.get(DcMotor.class, lfName);
        rightFront = hardwareMap.get(DcMotor.class, rfName);
        leftBack = hardwareMap.get(DcMotor.class, lbName);
        rightBack = hardwareMap.get(DcMotor.class, rbName);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void applyDTS(double drive, double turn, double strafe) {
        leftFront.setPower(drive - turn + strafe);
        rightFront.setPower(drive + turn - strafe);
        leftBack.setPower(drive - turn - strafe);
        rightBack.setPower(drive + turn + strafe);
    }

    public void applyDTS(DTS dts) { // function overloading
        applyDTS(dts.drive, dts.turn, dts.strafe);
    }
}
